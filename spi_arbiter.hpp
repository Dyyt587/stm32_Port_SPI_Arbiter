#ifndef SPI_BUS_H
#define SPI_BUS_H
/*****************************************************************//**
 * \file   spi_bus.hpp
 * \brief  spi总线的异步任务制驱动
 *
 * \author Dyyt
 * \date   December 2022
 *********************************************************************/
#define USE_OS 1
 //在阻塞（高实时）模式下的等待函数
#define SPI_IDLE_NONE ((void)0U)
#if USE_OS
//使用os时的非cpu阻塞（低实时）的等待函数
#define SPI_IDLE() osDelay(1) 
#else
//非使用os时的低实时时的等待函数
#define SPI_IDLE() ((void)0U)
#endif


#ifndef uint32_t
#define uint32_t unsigned int
#endif
#ifndef uint16_t
#define uint16_t unsigned short
#endif
#ifndef uint8_t
#define uint8_t unsigned char
#endif
typedef enum {
	LEVLE_NONE = (char)0x00,//无修改
	LEVLE_SOFT = (char)0x01,//仅软件
	LEVLE_SOFT_ALL = (char)0x02,//软件且包含DMA配置的影响
}Invalid_Levle;

typedef enum {
	SPI_MODE_IDLE,
	SPI_MODE_NONE
}SPI_Mode_t;
class SpiTask {
public:
	SpiTask(SPI_InitTypeDef* spi_cfg,
		void (*_spi_cs)(bool is_high),
		const uint8_t* tx_buf,
		uint8_t* _rx_buf,
		size_t _length,
		void (*_on_complete)(void*, bool),
		void* _on_complete_ctx) :
		SPI_Config(spi_cfg), spi_cs(_spi_cs), tx_buf(_tx_buf), rx_buf(_rx_buf), length(_length),
		on_complete(_on_complete), on_complete_ctx(_on_complete_ctx) {}
	//注意，并不会修改DMA相关配置，故不可改变传输位数！！！
	SPI_InitTypeDef* SPI_Config;//通常，一个应用程序只用一个配置，故采用指针，缩小内存占用

	void (*spi_cs)(bool is_high);//cs引脚
	const uint8_t* tx_buf;
	uint8_t* rx_buf;
	size_t length;
	void (*on_complete)(void*, bool);
	void* on_complete_ctx;

	inline void equals(const SPI_InitTypeDef& lhs, const SPI_InitTypeDef& rhs) {
		if ((lhs.Mode == rhs.Mode)
			&& (lhs.Direction == rhs.Direction)
			&& (lhs.DataSize == rhs.DataSize)
			&& (lhs.CLKPolarity == rhs.CLKPolarity)
			&& (lhs.CLKPhase == rhs.CLKPhase)
			&& (lhs.NSS == rhs.NSS)
			&& (lhs.BaudRatePrescaler == rhs.BaudRatePrescaler)
			&& (lhs.FirstBit == rhs.FirstBit)
			&& (lhs.TIMode == rhs.TIMode)
			&& (lhs.CRCCalculation == rhs.CRCCalculation)
			&& (lhs.CRCPolynomial == rhs.CRCPolynomial))
			invalid_levle = LEVLE_NONE;
		else if ((lhs.DataSize == rhs.DataSize))invalid_levle = LEVLE_SOFT;
		else invalid_levle = LEVLE_SOFT_ALL;
	}
private:
	Invalid_Levle invalid_levle = LEVLE_SOFT_ALL;//无效等级
	SpiTask* nextTask = nullptr;
	//bool is_in_use = false;
};

class SPI_Bus
{
public:
	SPI_bus(SPI_HandleTypeDef* hspi) : hspix(hspi) {}

	/**
	* @brief Enqueues a non-blocking transfer.
	*
	* Once the transfer completes, fails or is aborted, the callback is invoked.
	*
	* This function is thread-safe with respect to all other public functions
	* of this class.
	* 非阻塞的任务制spi总线传输函数
	*
	* @param task: Contains all configuration data for this transfer.
	*        The struct pointed to by this argument must remain valid and
	*        unmodified until the completion callback is invoked.
	*/
	bool transfer_async(SpiTask* task);

	/**
	 * @brief Executes a blocking transfer.
	 *
	 * If the SPI is busy this function waits until it becomes available or
	 * the specified timeout passes, whichever comes first.
	 *
	 * Returns true on successful transfer or false otherwise.
	 *
	 * This function is thread-safe with respect to all other public functions
	 * of this class.
	 *  该函数为线程阻塞制，等待时使用SPI_IDLE宏或获取信号量（系统）挂起
	 *
	 * @param config: The SPI configuration to apply for this transfer.
	 * @param ncs_gpio: The active low GPIO to actuate during this transfer.
	 * @param tx_buf: Buffer for the outgoing data to be sent. Can be null unless
	 *        rx_buf is null too.
	 * @param rx_buf: Buffer for the incoming data to be sent. Can be null unless
	 *        tx_buf is null too.
	 */
	bool transfer(SPI_InitTypeDef* config, void (*spi_cs_)(bool is_high), const uint8_t* tx_buf, uint8_t* rx_buf, size_t length, uint32_t timeout, SPI_Mode_t mode);

	/**
	 * @brief Completion method to be called from HAL_SPI_TxCpltCallback,
	 * HAL_SPI_RxCpltCallback and HAL_SPI_TxRxCpltCallback.
	 */
	void on_complete();

private:
	bool start();//内部调用，开始传输
	SPI_HandleTypeDef* hspix;

	volatile SpiTask* task_list_out = nullptr;//指向当前正在传输的任务
	volatile SpiTask* task_list_in = nullptr;//指向最后任务

	inline void __SPI_CFG()
	{
		/* Check the parameters */
		assert_param(IS_SPI_ALL_INSTANCE(hspix->Instance));
		assert_param(IS_SPI_MODE(hspix->Init.Mode));
		assert_param(IS_SPI_DIRECTION(hspix->Init.Direction));
		assert_param(IS_SPI_DATASIZE(hspix->Init.DataSize));
		assert_param(IS_SPI_NSS(hspix->Init.NSS));
		assert_param(IS_SPI_BAUDRATE_PRESCALER(hspix->Init.BaudRatePrescaler));
		assert_param(IS_SPI_FIRST_BIT(hspix->Init.FirstBit));
		assert_param(IS_SPI_TIMODE(hspix->Init.TIMode));
		if (hspix->Init.TIMode == SPI_TIMODE_DISABLE)
		{
			assert_param(IS_SPI_CPOL(hspix->Init.CLKPolarity));
			assert_param(IS_SPI_CPHA(hspix->Init.CLKPhase));
		}
#if (USE_SPI_CRC != 0U)
		assert_param(IS_SPI_CRC_CALCULATION(hspix->Init.CRCCalculation));
		if (hspix->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
		{
			assert_param(IS_SPI_CRC_POLYNOMIAL(hspix->Init.CRCPolynomial));
		}
#else
		hspix->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
#endif /* USE_SPI_CRC */
		__HAL_SPI_DISABLE(hspix);
		hspix->Init = *(task_list_out)->SPI_Config;
		/*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state */
		WRITE_REG(hspix->Instance->CR1, (hspix->Init.Mode | hspix->Init.Direction | hspix->Init.DataSize |
			hspix->Init.CLKPolarity | hspix->Init.CLKPhase | (hspix->Init.NSS & SPI_CR1_SSM) |
			hspix->Init.BaudRatePrescaler | hspix->Init.FirstBit | hspix->Init.CRCCalculation));

		/* Configure : NSS management */
		WRITE_REG(hspix->Instance->CR2, (((hspix->Init.NSS >> 16U) & SPI_CR2_SSOE) | hspix->Init.TIMode));

#if (USE_SPI_CRC != 0U)
		/*---------------------------- SPIx CRCPOLY Configuration ------------------*/
		/* Configure : CRC Polynomial */
		if (hspix->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
		{
			WRITE_REG(hspix->Instance->CRCPR, hspix->Init.CRCPolynomial);
		}
#endif /* USE_SPI_CRC */

#if defined(SPI_I2SCFGR_I2SMOD)
		/* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
		CLEAR_BIT(hspix->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif /* USE_SPI_CRC */

		hspix->ErrorCode = HAL_SPI_ERROR_NONE;
		hspix->State = HAL_SPI_STATE_READY;


	}
	inline bool __SPI_TRANSMIT()
	{
		HAL_StatusTypeDef status = HAL_ERROR;
		task_list_out->spi_cs(false);
		if (hspix->hdmatx->State != HAL_DMA_STATE_READY || hspix->hdmarx->State != HAL_DMA_STATE_READY) {
			// This can happen if the DMA or interrupt priorities are not configured properly.
			status = HAL_BUSY;//便于调试
		}
		else if (task_list_out->tx_buf && task_list_out->rx_buf) {
			status = HAL_SPI_TransmitReceive_DMA(hspix, (uint8_t*)task_list_out->tx_buf, task_list_out->rx_buf, task_list_out->length);
		}
		else if (task_list_out->tx_buf) {
			status = HAL_SPI_Transmit_DMA(hspix, (uint8_t*)task_list_out->tx_buf, task_list_out->length);
		}
		else if (task_list_out->rx_buf) {
			status = HAL_SPI_Receive_DMA(hspix, task_list_out->rx_buf, task_list_out->length);
		}

		if (status != HAL_OK) {
			task_list_out->spi_cs(true);
		}

		return status == HAL_OK;
	}
};

#endif // !SPI_BUS_H
