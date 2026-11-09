# Every internal quantity of one PID loop for one update, for plotting and debugging.

float64 reference
float64 measurement
float64 error        # wrapped to [-pi, pi] on the heading loop

float64 kp           # gains in use after gain scheduling
float64 ki
float64 kd

float64 p
float64 i
float64 d
float64 feedforward

float64 unsaturated  # p + i + d + feedforward
float64 output       # command sent to the allocation node
bool saturated       # the command was limited here or by the allocation node
