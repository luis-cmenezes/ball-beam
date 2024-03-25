import rclpy, os
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

from pot_calibration.client_pot import PotentiometerClient

def save_calibration(points, ang, lin):
    with open('/root/ball-beam/src/pot_calibration/calibration/current_calibration.txt') as file:
        for point in points:
            f.write(f'{point}\n')

        sign = '+' if lin >= 0 else '-'
        f.write(f'Angle={ang}*PotRead {sign} {abs(lin)}')
    
def plot_calibration(points, ang, lin):
    points = np.array(points)
    x = points[:,0]
    y = points[:,1]

    # plotting the actual points as scatter plot
    plt.scatter(x, y, color = "m",
            marker = "o", s = 15)

    # predicted response vector
    y_pred = lin + ang*x
    
    # plotting the regression line
    plt.plot(x, y_pred, color = "g")
    
    # putting labels
    plt.xlabel('Pot Read (0-4096)')
    plt.ylabel('Beam Angle (Rad)')
    sign = '+' if lin >= 0 else '-'
    plt.legend(['',f'Angle={round(ang,5)}*PotRead {sign} {round(abs(lin),5)}'])

    # saving image
    plt.savefig('/root/ball-beam/src/pot_calibration/calibration/current_calibration.png')
    plt.close()

def calibration(client):
    com = None

    calibration_points = []
    while com != 'q':
        print('Move up [U] | Move down [D] | Quit [Q]')
        com = input('Select option: ').lower()

        if com in 'ud':
            if com == 'u':
                servo_increment = 10
            elif com == 'd':
                servo_increment = -10
            
            pot = client.send_request(servo_increment).pot_read

            try:
                rad_angle = round( 3.14*float(input('Angle measurement (degrees): '))/180.0 , 5)
                calibration_points.append((pot, rad_angle))
            except:
                print('Input error, point not added to list.')

    return calibration_points

def lin_reg(points):
    points = np.array(points)
    x = points[:,0]
    y = points[:,1]

    slope, intercept, _, _, _ = stats.linregress(x, y)

    return slope, intercept

def main(args=None):
    rclpy.init(args=args)
    
    client = PotentiometerClient()

    cal_points =  calibration(client)
    print(f'Points captured (Pot-Read, Beam-Angle(rad)):\n\t{cal_points}')

    ang_coef, lin_coef = lin_reg(cal_points)

    plot_calibration(cal_points, ang_coef, lin_coef)
    
    save_calibration(cal_points, ang_coef, lin_coef)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()