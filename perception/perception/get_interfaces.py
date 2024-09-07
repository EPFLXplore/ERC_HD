import yaml

class GetInterfaces:
 
    @staticmethod
    def get_all():
        path = './src/custom_msg/config/hd_interface_names.yaml'
        with open(path, "r") as file:
            interfaces = yaml.safe_load(file)
        interfaces = interfaces['/**']['ros__parameters']
        print(interfaces)

    def get(interface):
        """
        Returns the topic value for the given topic name if it exists.
        """
        path = './src/custom_msg/config/hd_interface_names.yaml'
        with open(path, "r") as file:
            interfaces = yaml.safe_load(file)
        interfaces = interfaces['/**']['ros__parameters']
        if not interface in interfaces.keys():
            raise Exception(f"Interface: = {interface} = Does Not Exist!")
        return interfaces[interface]


if __name__ == "__main__":
    GetInterfaces.get()