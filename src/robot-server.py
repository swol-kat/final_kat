from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS


app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")






if __name__ == '__main__':
    print('[INFO] Starting server at http://localhost:6969')
    socketio.run(app=app, host='0.0.0.0', port=6969, debug=True)