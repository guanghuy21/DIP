from flask import Flask, jsonify, make_response
from api.routes import api_blueprint  # Import the routes
from flask_cors import CORS
import socket
app = Flask(__name__)
CORS(app)
# 1. Register the routes from routes.py
app.register_blueprint(api_blueprint)

# 2. Global Error Handlers (Kept here as they apply to the whole server)
@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({"error": "exitcode:404"}), 404)

@app.errorhandler(500)
def server_error(error):
    return make_response(jsonify({"error": "Internal Server Error"}), 500)

def start_api_gateway(host="127.0.0.1", port=3000):
    lan_ip = socket.gethostbyname(socket.gethostname())
    if lan_ip: host = lan_ip
    # print(f"API running at http://{lan_ip}:{port}")
    print(f"[*] API Gateway starting on http://{host}:{port}")
    # We disable the reloader because it can conflict with the Boss's threads
    app.run(host=host, port=port, debug=True, use_reloader=True)