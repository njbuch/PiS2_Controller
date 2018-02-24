######################################################################################
#
# RPI controller to be i2c master and react to the "alert-pin" from the Arduino-part
# of the controller.
# The Arduino part switches on the RPI when needed and this program starts on boot
# First state is sleep, and then checking the pin, if the pin is high, there is
# a message waiting (this happens on a "trigger" being activated)
# This program is primarily designed as a finite-state-machine to handle longer
# running processes and states, also handling the verified i2c communication.
#
#######################################################################################
#
# First version create 21-02-18 by Niels Jakob Buch
# Look at https://github.com/pytransitions/transitions for docs
# This fsm library also supports drawing a graph of the fsm, but getting
# required libraries to work under windows was too cumberzome. :(
#
######3###############################################################################

from time import sleep
from transitions.extensions import GraphMachine
from transitions.extensions.states import add_state_features, Timeout



# Set up logging; The basic log level will be DEBUG
import logging
logging.basicConfig(level=logging.DEBUG)
# Set transitions' log level to INFO; DEBUG messages will be omitted
# logging.getLogger('transitions').setLevel(logging.INFO)


## Message codes, should be verified and the same on the Arduino side
MSG_OK = 11
MSG_READY = 22

## The timeout is the waiting time until there is a timeout for a reply on the i2c bus
WAIT = 2


""" Use the decorator to add extensions """
@add_state_features(Timeout)
class CustomStateMachine(GraphMachine):
    pass


class RPI_Controller(object):
    def __init__(self):
        self.entourage = 0
        self.received_data = 0

    def on_enter_waking(self):
        data = request_data()
        print("Data received on request:", data)
        if data == MSG_READY:
            controller.rdy_received()
        if data != MSG_READY:
            controller.not_ready()

    def on_enter_state2(self):
        data = request_data()
        print("Data received on request in state2:", data)
        self.received_data = data
        controller.message_received()

    def on_enter_state3(self):
        send_data(self.received_data)
        data = request_data()
        if data == MSG_OK:
            controller.ok_received()
        else:
            controller.error

    def error_transition(self):
        print("Moving to sleep state because of error")

states = [{'name': 'sleep'},
          {'name': 'waking', 'timeout': WAIT, 'on_timeout': 'not_ready'},
          {'name': 'state2', 'timeout': WAIT, 'on_timeout': 'timeout1'},
          {'name': 'state3', 'timeout': WAIT, 'on_timeout': 'timeout2'},
          {'name': 'state4'}
          ]

#                Name, From, To
transitions = [['pin_high', 'sleep', 'waking'],
               ['not_ready', 'waking', 'waking'],
               ['rdy_received', 'waking', 'state2'],
               ['message_received', 'state2', 'state3'],
               ['ok_received', 'state3', 'state4'],
               ['pin_low', 'state4', 'sleep'],
               ['timeout1', 'state2', 'sleep'],
               ['timeout2', 'state3', 'sleep']]

controller = RPI_Controller()
machine = CustomStateMachine(model=controller, states=states, transitions=transitions, initial='sleep')
machine.add_transition('error', source='*', dest='sleep', before='error_transition')

#######################################################################
# these functions are doing the primary communication on the i2c bus
# request_data is the masters way to request data from slaves
# there is no specific attribute with the call, essentially its a one-way
# from the the slave, to the master
def request_data():
    print("Data requested in ", controller.state, " state.")
    if controller.state == 'waking':
        sleep(15)   # this is to test that the timeout works
        return MSG_READY
    if controller.state == 'state3':
        return MSG_OK
    return 253

# send_data essentially sends data to the slave, with no immediate response from the slave
#
def send_data(data):
    print("Now sending data:", data)


print("Starting...")
print("state:", controller.state)
# controller.pin_high();
print("state:", controller.state)





# assert controller.state == 'sleep'  # Preparing for the night shift
# assert machine.get_state(controller.state).is_busy  # We are at home and busy
# print(machine.get_state(controller.state).is_busy)
# controller.done()
# assert controller.state == 'waiting'  # Waiting for fellow supercontrolleres to join us
# assert controller.entourage == 1  # It's just us so far
# sleep(0.7)  # Waiting...
# controller.join()  # Weeh, we got company
# sleep(0.5)  # Waiting...
# controller.join()  # Even more company \o/
# sleep(2)  # Waiting...
# assert controller.state == 'away'  # Impatient supercontroller already left the building
# assert machine.get_state(controller.state).is_home is False  # Yupp, not at home anymore
# assert controller.entourage == 3  # At least he is not alone

# machine = Machine(model=m, show_auto_transitions=True, states=states, transitions=transitions)
# in cases where auto transitions should be visible
# Machine(model=m, show_auto_transitions=True, ...)


""" machine2 = GraphMachine(model=controller,
                       states=states,
                       transitions=transitions,
                       auto_transitions=False,
                       initial='sleep',
                       title="Mood Matrix",
                       show_conditions=True)
"""

# draw the whole graph ...
controller.get_graph().draw('my_state_diagram.png', prog='dot')
# ... or just the region of interest
# (previous state, active state and all reachable states)
# m.get_graph(show_roi=True).draw('my_state_diagram.png', prog='dot')
