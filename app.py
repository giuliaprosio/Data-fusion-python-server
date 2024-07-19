# python.py
# import Flask and create a Flask application instance 
from flask import Flask, request, jsonify
from flask_cors import CORS
from LSL import *
from ukf import *
import json

app = Flask(__name__)
CORS(app)

filter = None

# define a route '/run-python-script' that will execute the python script
@app.route('/run-python-script', methods=['POST'])

def run_python_script():

    """" run the application. Invoked when receiving a POST message 
    Takes the json object received, 
    Calculates for the object first the Least Square Lateration of each node 
    Then smoothes the result using Unscented Kalman Filters
    Returns as the response of the POST request the data modified. 
    """
    global filter
    
    data_received = request.json 

    nodes = data_received['nodes']
    if(len(nodes) <= 3): 
        return jsonify(data_received)
    
    lsl(data_received)
    if not filter:  
        filter = Filter([n["associatedRouterName"] for n in nodes], nodes)
        filter.initial_state(nodes)
    else:
        print("hoal")
        filter.apply_filter(nodes)

    with open('output.json', 'a') as f:
        json.dump(data_received, f, indent=4)
        f.write(',')

    return jsonify(data_received)

if __name__ == '__main__':
    app.run(debug = True)