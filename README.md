The purpose of this repo is to make it easy to see what the sensors on the network see and what their ip address is. This makes it easier to assign sensors when setting them up to use Moetsi's RaaS.

To set up and run:


1. `python3 -m venv oakd-env`
2. `source oakd-env/bin/activate`  # On Windows, use `oakd-env\Scripts\activate`
3. `pip install -r requirements.txt`
4. `python see_all_available_sensors.py`
