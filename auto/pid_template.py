import numpy as np
import math

degress_to_radians = 0.0174533
g = 9.81

def make_car(desired_v:float=20, dt:float=0.1) -> dict:
    """
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
        "angle" : 0,

        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = .99, angle_speed: float = .4) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = ((force / mass)*friction - g*math.sin(degress_to_radians * car["angle"]))

        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        if car["t"] > 100 and car["t"] < 120:
                car["angle"] += angle_speed * car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0, en_feedforward: bool = False) -> tuple[float, float]:
        #input: car["v"], car["desired_v"] (floats)
        #output: desired acceleration and error tuple(float, float)
        # pass # delete this line and write your PID code here
        error = car["desired_v"] - car["v"]
        car["net_integral"] += error
        desired_acc = K_P * error + K_I * car["net_integral"]
        if car["error_prev"] != None:
                desired_acc += K_D * ((error - car["error_prev"])/car["dt"])
        if en_feedforward:
                feed_forward = g*math.sin(degress_to_radians * car["angle"])
                desired_acc += feed_forward
        car["error_prev"] = error
        return (desired_acc, error)


def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        #input: desired_acceleration(float)
        #output: throttle percentage (float, -1 to 1)
        # pass # delete this line and write your code to convert desired acceleration to throttle here
        max_acc = max_throttle_force/mass
        np.clip(acceleration_desired, -max_acc, max_acc)
        # print(str( np.interp(acceleration_desired, [-max_acc, max_acc], [-1, 1])))
        return np.interp(acceleration_desired, [-max_acc, max_acc], [-1, 1])