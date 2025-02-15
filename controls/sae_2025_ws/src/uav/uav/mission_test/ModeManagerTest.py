from time import time
import yaml

#TODO: Think about how to encode the mission structure (when to switch `Modes`, etc.)
class ModeManager():    
    """
    A ROS 2 node for managing UAV modes and mission logic.
    """

    def __init__(self, mode_map: str):
        self.modes = {}
        self.transitions = {}
        self.active_mode = None
        self.last_update_time = time()
        self.starting_mode = 'start'

        self.setup_modes(mode_map)

        self.on_enter()

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        """
        self.switch_mode(self.starting_mode)
        
    def setup_modes(self, mode_map: str) -> None:
        """
        Setup the modes for the mission node.

        Args:
            mode_yaml (dict): A dictionary mapping mode names to instances of the mode.
        """
        mode_yaml = self.load_yaml_to_dict(mode_map)

        for mode_name in mode_yaml.keys(): 
            params = mode_yaml[mode_name].get('params', {})    
            for key, value in params.items():
                params[key] = eval(value)
            self.modes[mode_name] = params
            
            self.transitions[mode_name] = mode_yaml[mode_name].get('transitions', {})

    def transition(self, state: str) -> str:
        """
        Transition to the next mode based on the current state.

        Args:
            state (str): The current state of the mode.

        Returns:
            str: The name of the next mode to transition to.
        """

        if state in self.transitions[self.active_mode]:
            print(f"Transitioning to {self.transitions[self.active_mode][state]}")
            self.transitions[self.active_mode][state]
        else:
            print(f"Transition {state} not found in mode {self.active_mode}")

    def switch_mode(self, mode_name: str) -> None:
        """
        Switch to a new mode.

        Args:
            mode_name (str): Name of the mode to activate.
        """
        if self.active_mode:
            print('deactivating mode')

        if mode_name in self.modes:
            self.active_mode = mode_name
            print('activating mode')
        else:
            print(f"Mode {mode_name} not found.")

    def load_yaml_to_dict(self, filename: str):
        """
        Load a yaml file into a dictionary.

        Args:
            filename (str): The path to the yaml file.

        Returns:
            dict: The yaml file as a dictionary.
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return data
    

if __name__ == '__main__':
    mission_node = ModeManager('./missions/sim_test.yaml')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)

    mission_node.transition('continue')
    print(mission_node.active_mode)