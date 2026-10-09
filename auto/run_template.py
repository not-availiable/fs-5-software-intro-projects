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

data_points = ["time_data","process_data","error_data","angle_data", "desired_acc"]
target_data = ["process_setpoint"]
TIME = 40
DT = .1
STEPS = int(TIME/DT)
#pre initialize the arrays to make the slider/button callbacks faster
with_feedforward_data = {d: [0] * STEPS for d in data_points}
without_feedforward_data = {d: [0] * STEPS for d in data_points}
position_control = False

def write_data(container: dict, data: list, index:int) -> None:
    """
    Writes the sim data to a dictionary

    Inputs:
    Container: the dictionary to write to
    Data: each index i of data should correspond to container[data_points[i]][index]
    Index: if index is -1 it means that the each data value should be treated as a constant rather than an entry into one of the pre initialized lists,
    ie: storing the setpoint so it can be drawn. Otherwise index represents which index of the pre-initialized arrays is currently being written
    """
    if index == -1:
        for key, value in zip(target_data, data):
            container[key] = value
    else:
        for key, value in zip(data_points, data):
                container[key][index] = value

def run_sim(label:str="") -> None:
    """
    Runs each car through a simulation STEPS long, writes the data to graphs, and tells the drawing file to draw said data to the screen

    Inputs:
    label, does nothing but the sim won't update on the button press unless this parameter exists, thanks matplotlib :(
    """
    position_control = get_control_mode()
    K_P, K_I, K_D, velocity_setpoint, position_setpoint = get_slider_values()
    write_data(with_feedforward_data, [position_setpoint if position_control else velocity_setpoint], -1)
    write_data(without_feedforward_data, [position_setpoint if position_control else velocity_setpoint], -1)
    car = make_car(desired_v=velocity_setpoint, dt=0.1, desired_x=position_setpoint, position_control=position_control, angle_start_time=TIME/3, angle_end_time=(2*TIME/3))
    carNF = make_car(desired_v=velocity_setpoint, dt=0.1, desired_x=position_setpoint, position_control=position_control, angle_start_time=TIME/3, angle_end_time=(2*TIME/3))

    for i in range(STEPS):
        desired_acc, error = calculate_desired_acceleration(car, K_P, K_I, K_D, True)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(car, desired_throttle)

        write_data(with_feedforward_data, [car["t"], car["x"] if position_control else car["v"], error, car["angle"], desired_throttle],i)

        desired_acc, error = calculate_desired_acceleration(carNF, K_P, K_I, K_D, False)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(carNF, desired_throttle)

        write_data(without_feedforward_data, [carNF["t"], carNF["x"] if position_control else carNF["v"], error, carNF["angle"], desired_throttle],i)
    draw_all(with_feedforward_data, without_feedforward_data)
init_graph(run_sim)
run_sim()
plt.show()