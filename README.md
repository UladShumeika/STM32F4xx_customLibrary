# STM32F4xx_customLibrary
At the moment it is not a complete library like HAL or SPL. The purpose of this library is to understand in detail how to work with the controller's peripherals. As needed, I add new drivers and features.

**Index:**
 - [Library content](https://github.com/UladShumeika/STM32F4xx_customLibrary/tree/main#library-content)
 - [How to use](https://github.com/UladShumeika/STM32F4xx_customLibrary/tree/main#how-to-use)
 - [Further possible improvements](https://github.com/UladShumeika/STM32F4xx_customLibrary/tree/main#further-possible-improvements)

## Library content
- MISC driver
- RCC driver
- GPIO driver
- DMA driver
- SPI driver
- UART driver
- I2C driver
- CAN driver

## How to use
- сonnect **ush_stm32f4xx_conf.h** file to your project;
- comment out/uncomment required drivers in **ush_stm32f4xx_conf.h** file.
  ```
  /*!
  * @name custom_drivers
  * @{
  */
  #define PRJ_MISC_DRIVER				/*!< Miscellaneous driver */
  #define PRJ_RCC_DRIVER				/*!< RCC driver */
  #define PRJ_GPIO_DRIVER				/*!< GPIO driver */
  #define PRJ_DMA_DRIVER 				/*!< DMA module */
  /* #define PRJ_SPI_DRIVER  */	                /*!< SPI driver */
  /* #define PRJ_UART_DRIVER */	                /*!< UART driver */
  #define PRJ_I2C_DRIVER				/*!< I2C driver */
  #define PRJ_CAN_DRIVER				/*!< CAN driver */

  /*! @}*/
  ```
## Further possible improvements
Since the drivers are obviously raw, then all the improvements can be listed indefinitely, however, I would like to note the following general changes:
- redesign of SPI and UART drivers, since at the time of their design it seemed to me a great idea to add initialization of auxiliary peripherals such as GPIO and DMA inside. Because of this, the initialization functions turned out to be unnecessarily complex and long;
- bringing all drivers to a single code style such as in I2C and DMA drivers, as well as getting rid of unnecessary enumerations;
- add error handling
