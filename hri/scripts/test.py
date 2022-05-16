from pathlib import Path
data_folder = Path("/home/stijn/tiago_dual_public_ws/src/cor_mdp_tiago/hri/speeches")
file = data_folder / "introduction.txt"
with open(file, 'r') as file:
    data = file.read().replace('\n', ' ')
print(data)


