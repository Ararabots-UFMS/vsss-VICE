from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
import math

app = Flask(__name__)
socketio = SocketIO(app,  cors_allowed_origins="*")

@socketio.on("connect")
def handle_connect():
    print("Client connected")
    global thread
    global stop_thread
    stop_thread = False
    if thread is None:
        thread = Thread(target=background_thread)
        thread.start()

@socketio.on("disconnect")
def handle_disconnect():
    global thread
    global stop_thread
    stop_thread = True
    thread = None

@socketio.on("fieldSide")
def handle_field_side(side):
    #TODO get side to rest of the code
    if side == True:
        print("Field is on the left side", flush=True)
    elif side == False:
        print("Field is on the right side", flush=True)

@socketio.on("teamColor")
def handle_field_side(color):
    #TODO get side to rest of the code
    if color == True:
        print("Team color is blue", flush=True)
    elif color == False:
        print("Team color is yellow", flush=True)

@socketio.on("message")
def handle_message(message):
    print(f"Received message: {message}", flush=True)

def background_thread():

    x = 0
    y = 0
    radius = 50
    angle = 0

    while not stop_thread:
        x = 250+radius * math.cos(angle)
        y = 250+radius * math.sin(angle)
        angle += 0.1
        socketio.emit("position", {"x": x, "y": y, "angle": angle})
        # print(f"Sent update: x={x}, y={y}", flush=True)
        socketio.sleep(0.1)  # Adjust the delay as needed

def main():
    global thread
    thread = None
    socketio.run(app, debug=True)

if __name__ == "__main__":
    main()