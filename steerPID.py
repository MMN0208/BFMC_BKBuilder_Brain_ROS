import math
import sys
def steerPID(input: float) -> [float, float]:
    
    error_integral = 0
    previous_error = 0

    Ki = 0.4
    Kp = 0.2
    Kd = 0.1
    
    processed = input
    setpoint  = float(math.floor(input))

    error = abs(setpoint - processed)

    error_integral = error_integral + error
    error_derivative = error - previous_error
    
    output = Kp * error + Ki*error_integral + Kd * error_derivative

    return error, error_integral, output

if __name__ == '__main__':

    error, output = steerPID(float(sys.argv[1]))

    print(error, output)
