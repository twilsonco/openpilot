#!/opt/homebrew/bin/python3

import matplotlib.pyplot as plt
import math

def dampen(x, exp):
    abs_x = abs(x)
    if abs_x <= 1.0:
        return x
    sign = math.copysign(1, x)
    return sign * (abs_x**exp)

x_values = [-3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3]

exponents = [0.8, 0.85, 0.9, 0.95, 0.99]

for exp in exponents:
    y_dampened = [dampen(x, exp) for x in x_values]
    plt.plot(x_values, y_dampened, label=f'Dampened, exp={exp}')

y_linear = x_values  
plt.plot(x_values, y_linear, label='Linear')

plt.xlabel('x')
plt.ylabel('f(x)')
plt.title('Dampening Function Comparison')
plt.legend()
plt.show()