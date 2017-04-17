#!/usr/bin/env python
from flask import render_template


# init and start flask
from . import app




@app.route('/')
def index():
    return render_template('index.html')


@app.route('/map')
def map():
    return render_template('map.html')




if __name__ == "__main__":
    app.run()
