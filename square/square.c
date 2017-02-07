/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_config.h>


#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>  // hwtest
#include <uORB/uORB.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__EXPORT int	square_main(int argc, char *argv[]);

int
square_main(int argc, char *argv[])
{
        
    struct actuator_controls_s actuators; 
    memset(&actuators, 0, sizeof(actuators));
    orb_advert_t actuator_pub_ptr = orb_advertise(ORB_ID(actuator_controls_0), &actuators); // объявление издателя actuator_pub_ptr, содержащего функцию (orb_advertise) объявления топика "actuator_controls_0" и размещения начальной публикации (&actuators), ORB_ID - указатель на топик.

    struct actuator_armed_s arm; 
    memset(&arm, 0 , sizeof(arm));

    arm.timestamp = hrt_absolute_time(); //ЭТО
    arm.ready_to_arm = true;
    arm.armed = true;
    orb_advert_t arm_pub_ptr = orb_advertise(ORB_ID(actuator_armed), &arm); // объявление издателя arm_pub_ptr, аналогично
    orb_publish(ORB_ID(actuator_armed), arm_pub_ptr, &arm); // функция публикации в ранее созданный топик (actuator_armed) ... зачем ведь только что было сделано то же самое?

    /* read back values to validate */ // чтение данных для проверки
    int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed)); // создание переменной равной результату выполнения функции подписки на топик
    orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm); // копирование

    if (arm.ready_to_arm && arm.armed) {
        warnx("Actuator armed");

    } else {
        errx(1, "Arming actuators failed");
    }


   for (int i = 0; i != 4; i++) {
     
    while ((int ch = getopt(argc - 1, &argv[1], "f:t:")) != EOF) { //why int ch & argc-1?
        switch (ch) {

        //fpwm

        case 'f':
            actuators.control[0] = 1.5f;
            actuators.control[1] = 1.0f;
            sleep(optarg);
            break;

        case 't':
            actuators.control[0] = 1.3f;
            actuators.control[1] = 1.5f;
            sleep(optarg);
            break;

        }
        actuators.timestamp = hrt_absolute_time(); //ЭТО
        orb_publish(ORB_ID(actuator_controls_0), actuator_pub_ptr, &actuators); // публикуем в топик actuator_controls_0 то, что у нас в структуре actuators(видимо, значение из actuators.control[i])
    }
   }

    return OK;
}



