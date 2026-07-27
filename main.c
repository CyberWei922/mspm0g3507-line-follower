#include "ti_msp_dl_config.h"

#include "app_config.h"
#include "APP/app_state_machine.h"
#include "BSP/Buzzer/buzzer.h"
#include "BSP/System/system_time.h"

int main(void)
{
    /* SysConfig是所有引脚、时钟和外设配置的唯一来源。 */
    SYSCFG_DL_init();

    /* 1 ms系统节拍同时驱动非阻塞蜂鸣器和协作式调度器。 */
    SystemTime_Init();
    Buzzer_Init();

#if !APP_ENABLE_BUZZER_OUTPUT
    /*
     * 保留蜂鸣器软件状态机，只在最外层断开PB24物理输出。
     * PWM定时器停止且引脚切为高阻输入，因此后续提示逻辑不会发声。
     */
    DL_Timer_stopCounter(BUZZER_INST);
    DL_GPIO_disableOutput(GPIO_BUZZER_C3_PORT, GPIO_BUZZER_C3_PIN);
    DL_GPIO_initDigitalInput(GPIO_BUZZER_C3_IOMUX);
#endif

    /* 初始化、自检和所有状态逻辑均封装在APP层。 */
    AppStateMachine_Init();

    for (;;) {
        AppStateMachine_Run();
        __WFI();
    }
}
