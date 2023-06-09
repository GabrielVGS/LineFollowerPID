import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *
import time
import math


def follow_for_forever(tank):
    """
    ``tank``: the MoveTank object that is following a line
    """
    return True

class PID(MoveSteering):
    def __init__(self,left_motor_port, right_motor_port,desc=None,motor_class=LargeMotor) -> None:
        MoveSteering.__init__(self, left_motor_port, right_motor_port, desc, motor_class)
        self.cs = None
    
    def follow_line_color(self,
                        target,
                        speed,
                        follow_for = follow_for_forever,
                        white=99,
                        off_line_count_max=20,
                        sleep_time=0.01,
                        kp=0,
                        ki = 0,
                        kd = 0,
                        debug=False,
                        **kwargs):
        

        if self.cs is None:
            raise Exception("Color sensor not set")
        

        integral = 0
        speed = speed

        while follow_for(self, **kwargs):
            reflected_light_intensity = self.cs.reflected_light_intensity
            error = reflected_light_intensity - target
            last_error = error
            integral = integral + error
            derivative = error - last_error

            turn = (kp * error) + (ki * integral) + (kd * derivative)
            turn = max(min(turn,100),-100)

            if reflected_light_intensity >= white:
                off_line_count += 1
                
                if off_line_count >= off_line_count_max:
                    break
            else:
                off_line_count = 0
            
            if sleep_time:
                time.sleep(sleep_time)

            try:
                if debug:
                    self.on(turn, speed)
                    print(f"Cor {reflected_light_intensity}, steering {turn}, speed {speed}")
                else:
                    self.on(turn, speed)
            except Exception as e:
                print(e)
                self.stop()
        self.stop()






pen = Pen(INPUT_5)
pen.setColor(*(1,0,0))
pen.down()
pid = PID(OUTPUT_A,OUTPUT_B)
pid.cs = ColorSensor(INPUT_1)
pid.follow_line_color(45/2,20,kp=5.75,ki=0.00927,kd = 0.6245)
