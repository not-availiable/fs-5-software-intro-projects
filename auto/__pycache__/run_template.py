import matplotlib
matplotlib.use('qtagg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Slider
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage
from drawing import init_graph
from drawing import get_slider_values
from drawing import draw_graph

vel_data = []
vel_data_wo_ff = []
error_data = []
error_data_wo_ff = []
time_data = []
angle_data = []

# K_P = .07
# K_I = .00007
# K_D = 0

STEPS = 2000

def run_sim(val):
    car = make_car(desired_v=40, dt=0.1)
    carNF = make_car(desired_v=40, dt=0.1)

    vel_data.clear()
    vel_data_wo_ff.clear()
    error_data.clear()
    error_data_wo_ff.clear()
    time_data.clear()
    angle_data.clear()

    K_P, K_I, K_D = get_slider_values()

    for i in range(STEPS):
        desired_acc, error = calculate_desired_acceleration(car, K_P, K_I, K_D, True)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(car, desired_throttle)

        error_data.append(error)

        desired_acc, error = calculate_desired_acceleration(carNF, K_P, K_I, K_D, False)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(carNF, desired_throttle)

        vel_data.append(car["v"])
        vel_data_wo_ff.append(carNF["v"])
        error_data_wo_ff.append(error)
        time_data.append(car["t"])
        angle_data.append(car["angle"])

run_sim(0)
plt.show()
