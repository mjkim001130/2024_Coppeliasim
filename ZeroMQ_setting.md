

## Install 

```shell
pip install coppeliasim-zmqremoteapi-client
```

## Simple example

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class class_name():
	def __init__(self):
		self.client = RemoteAPIClient()
		self.sim = self.client.require('sim')

	def run(self):
		self.sim.setStepping(True)
        	self.sim.startSimulation()
        	self.sim.stopSimulation()

if __name__ == "__main__":
	class_name = class_name()
	class_name.run()
```

