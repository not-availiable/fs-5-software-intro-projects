import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('qtagg')
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage
from drawing import init_graph
from drawing import get_slider_values
from drawing import get_control_mode
from drawing import draw_all


data_points = ["time_data","vel_data","error_data","angle_data"]
STEPS = 550
with_feedforward_data = {d: [0] * STEPS for d in data_points}
without_feedforward_data = {d: [0] * STEPS for d in data_points}
position_control = False

def write_data(container: dict, data: list, index:int) -> None:
    for key, value in zip(data_points, data):
        container[key][index] = value;

def run_sim(label:str="") -> None:
    position_control = get_control_mode()
    car = make_car(desired_v=10, dt=0.1, position_control=position_control)
    carNF = make_car(desired_v=10, dt=0.1, position_control=position_control)

    K_P, K_I, K_D = get_slider_values()

    for i in range(STEPS):
        desired_acc, error = calculate_desired_acceleration(car, K_P, K_I, K_D, True)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(car, desired_throttle)

        write_data(with_feedforward_data, [car["t"], car["x"] if position_control else car["v"], error, car["angle"]],i)

        desired_acc, error = calculate_desired_acceleration(carNF, K_P, K_I, K_D, False)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(carNF, desired_throttle)

        write_data(without_feedforward_data, [carNF["t"], carNF["x"] if position_control else carNF["v"], error, carNF["angle"]],i)
    draw_all(with_feedforward_data, without_feedforward_data)

init_graph(run_sim)
run_sim()
plt.show()