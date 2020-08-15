# header, which should include things like time stamp data.
Header header

# the state of the pin. true means high, false means low.
bool state

# the pin that has changed. Represented as a string because
# some pins are named things like '12', while others are named
# things like 'P8' depending on your hardware.
string pin