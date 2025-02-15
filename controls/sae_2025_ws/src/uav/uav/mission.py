from ModeManager import ModeManager

def main():
  mission = ModeManager('./missions/sim_test.yaml')
  print(mission.modes)
  print(mission.active_mode)
  print(mission.transitions)
  mission.transition('continue')
  print(mission.active_mode)
 
if __name__ == '__main__':
  main()
