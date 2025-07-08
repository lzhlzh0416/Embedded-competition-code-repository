#include "buzzer_task.hpp"
#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);
float HZ_1 = 392.00;
float HZ_2 = 493.88;
float HZ_3 = 440.00;
// 261.63 329.63 392
extern "C" void buzzer_task()
{
    buzzer.set(HZ_1, 0.1);
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);

    buzzer.set(HZ_2, 0.1);
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);

    buzzer.set(HZ_3, 0.1);
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);

    while (true)
    {
        osDelay(100);
    }
}

// // 校准提示音
void calibrate_buzzer(void)
{
    for (int i = 0; i < 3; i++)
    {
        buzzer.start();
        osDelay(100);
        buzzer.stop();
        osDelay(100);
    }
}

void error_buzzer(void)
{
    for (int i = 0; i < 3; i++)
    {
        buzzer.start();
        osDelay(100);
        buzzer.stop();
        osDelay(100);
    }
}
