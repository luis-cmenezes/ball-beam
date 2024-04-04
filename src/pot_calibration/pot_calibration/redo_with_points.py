import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def save_calibration(points, ang, lin):
    with open('/root/ball-beam/src/pot_calibration/calibration/current_calibration.txt', 'a') as file:
        for point in points:
            file.write(f'{point}\n')

        sign = '+' if lin >= 0 else '-'
        file.write(f'Angle={ang}*PotRead {sign} {abs(lin)}')

def lin_reg(points):
    points = np.array(points)
    x = points[:,0]
    y = points[:,1]

    slope, intercept, _, _, _ = stats.linregress(x, y)

    return slope, intercept

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

def calibration():
    return [(2310, -0.199), (2316, -0.166), (2335, -0.133), (2381, -0.085), (2426, -0.045), (2481, 0.005), (2524, 0.044), (2569, 0.091), (2622, 0.145), (2661, 0.181), (2701, 0.23), (2738, 0.263), (2733, 0.229), (2696, 0.183), (2657, 0.145), (2606, 0.089), (2561, 0.047), (2534, 0.007), (2475, -0.045), (2443, -0.085), (2389, -0.129), (2357, -0.16)]

def main(args=None):
    cal_points =  calibration()

    ang_coef, lin_coef = lin_reg(cal_points)

    plot_calibration(cal_points, ang_coef, lin_coef)
    
    save_calibration(cal_points, ang_coef, lin_coef)

if __name__ == '__main__':
    main()