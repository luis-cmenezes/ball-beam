import rclpy, os, csv
from datetime import date

from servo_beam_step.step_ros import StepROS

def step_exp_data(ros_obj, com):
    ros_obj.send_request(com)
    while not ros_obj.check_status():
        rclpy.spin_once(ros_obj)
                
    return ros_obj.return_data()

def save_csv(step, time, servo, angle):
    data = zip(time,servo,angle)

    file_name = f'{date.today()}_{step}_dg.csv'

    with open(f'/root/ball-beam/src/servo_beam_step/data/{file_name}','w',newline='') as csv_file:
        writer =  csv.writer(csv_file)

        writer.writerow(["Time [s]", "Servo [rad]", "Table angle [rad]"])

        writer.writerows(data)
    
    print(f'Test finished, data saved in {file_name} in the data folder of the package.')

def main(args=None):
    rclpy.init(args=args)

    ros_step = StepROS()
    
    com = None
    while rclpy.ok() and com != 'q':
        com = input('Servo step [angle]|Quit [q]: ')
        
        try:
            if com != 'q':
                step, time_vec, servo_vec, angle_vec = step_exp_data(ros_step, int(com))

                save_csv(step, time_vec, servo_vec, angle_vec)

        except Exception as e:
            print(e)
            print(f'{com} is not a valid input.')
        
    ros_step.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()