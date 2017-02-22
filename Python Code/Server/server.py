from datetime import datetime
from flask import Flask
from flask_restful import Resource, Api, reqparse

app = Flask(__name__)
api = Api(app)

class RootView(Resource):
    def get(self):
        return {'msg': 'hello world'}

    def post(self):
        parser = reqparse.RequestParser()
        parser.add_argument('temp', type=float, help='Temperature reading as integer')
        parser.add_argument('ambientLight', type=float, help='Ambient light reading as integer')
        parser.add_argument('humidity', type=float, help='Humidity reading as integer')
        args = parser.parse_args()

        # Get the values we want
        temp = str(args['temp'])
        light = str(args['ambientLight'])
        humidity = str(args['humidity'])

        # Get and prep time string
        time = str(datetime.now())
        time = time.split('.')
        time = time[0]

        # Make the CSV line
        csv_line = time + ',' + temp + ',' + light + ',' + humidity + '\n'

        # Write values to disk
        raw = open('results.csv', 'a')
        raw.write(csv_line)
        raw.close()

        # Save results to disk
        return {'msg': 'success'}

api.add_resource(RootView, '/')

if __name__ == '__main__':
    app.run(debug=True)
