#include "spi_bus.h"
/*****************************************************************//**
 * \file   spi_bus.cpp
 * \brief  spi总线的异步任务制驱动
 *
 * \author Dyyt
 * \date   December 2022
 *********************************************************************/
 /**
  * version :
  * 1.3 修改优化，transfer增加阻塞选择
  * 1.2 删除fifo，新增一个指针避免递归搜索
  * 1.1 新增加fifo和链表，且可选择
  * 1.0 初次创建
  */
  /**
   * 如何使用：
   * *一，如果使用HAL库*************************
   *		1.先对class进行初始化
   *		eg: SPI_Bus spi1(&hspi1);
   *		2.1使用transfer函数
   *		eg: transfer(...);
   *		2.2使用transfer_async函数
   *		eg: SpiTask task(...);//建议每一个线程（任务）各使用一个，且定义成全局变量方便使用，后续仅需修改buf和length即可
   *			transfer_async(&task);
   * *二，如果使用其他库***********************
   *		1.移植库
   *		2.使用该库
   * *三，移植
   * *	1.修改class中的的句柄（主要时为了2服务，也可直接删除）
   *		2.修改__SPI_CF()和__SPI_TRANSMIT()
   *		3.修改SpiTask中的SPI_Config和equals函数
   *		4.修改transfer函数的SPI_InitTypeDef* config
   *
   */
bool SPI_Bus::transfer_async(SpiTask* task)
{
	if (task_list_in == nulptr)
	{
		task->equals(task->SPI_Config, hspix->Init);
		restrict task_list_in = task;
		restrict task_list_out = task;
		if (!start()) return false;

	}
	else {
		task->equals(task->SPI_Config, task_list_in->SPI_Config);
		task_list_in->nextTask = task;
		task_list_in = task;
	}
	return true;

}

bool SPI_Bus::transfer(SPI_InitTypeDef* config, void (*spi_cs_)(bool is_high), const uint8_t* tx_buf, uint8_t* rx_buf, size_t length, uint32_t timeout, SPI_Mode_t mode)
{
	uint32_t result = 0xff;
	SpiTask task = {
	.config = config,
	.spi_cs = spi_cs_,
	.tx_buf = tx_buf,
	.rx_buf = rx_buf,
	.length = length,
	.on_complete = [](void* ctx, bool success) { *(volatile uint8_t*)ctx = success ? 1 : 0; },
	.on_complete_ctx = (void*)&result,
	.nextTask = nullptr

	};
	if (!SPI_Bus::transfer_async(&task)) return false;
	unt32_t time_cnt = 0;
	switch (mode)
	{
	case SPI_MODE_IDLE:
		while (result == 0xff)
		{
			if (time_cnt == timeout_ms)break;
			SPI_IDLE;
			time_cnt++;
		}
		break;
	case SPI_MODE_NONE:
		while (result == 0xff)
		{
			if (time_cnt == timeout_ms)break;
			SPI_IDLE_NONE;
			time_cnt++;
		}
	default:
		break;
	}

	return true;
}
//中断中使用
void SPI_Bus::on_complete(void)
{
	if (task_list_out == nullptr) return; // this should not happen
	task_list_out->spi_cs(true);
	task_list_out->on_complete(tasks[task_list_]->on_complete_ctx, true);
	//fifo的task_list_+1
	if (task_list_out->nextTask != nullptr)
	{
		task_list_out = task_list_out->nextTask;
		start();
	}
	else {
		restrict task_list_in = nullptr;
		restrict task_list_out = nullptr;
	}
}

bool SPI_Bus::start()
{

	switch (task_list_out->invalid_levle)
	{
	case LEVLE_NONE:
		break;
	case LEVLE_SOFT://不改变dma
		//几乎来自于HAL，简化配置，但也降低了安全性，用户必须保障传入数据得正确性
		__SPI_CFG();
		__HAL_SPI_ENABLE(hspix);
		break;
	case LEVLE_SOFT_ALL:
		HAL_SPI_DeInit(hspix);
		hspi_->Init = task_list_out->config;
		HAL_SPI_Init(hspix);
		__HAL_SPI_ENABLE(hspix);
		break;
	default:
		while (1);//不应该出现
		break;
	}
	task_list_out->spi_cs(false);
	return __SPI_TRANSMIT();

}


