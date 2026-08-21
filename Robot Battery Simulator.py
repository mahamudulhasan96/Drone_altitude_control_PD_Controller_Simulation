battery = 100
x = 0
y = 0

print("Robot Battery Simulator")
print("Use: w = up, s = down, a = left, d = right")
print("Type q to quit.\n")

while battery > 0:
    print(f"Robot Position: ({x}, {y})")
    print(f"Battery: {battery}%")

    move = input ("Move: ").lower()

    if move == "w":
        y += 1
    elif move == "s":
        y -= 1
    elif move == "a":
        x -= 1
    elif move == "d":
        x += 1
    elif move  == "q":
        print("Mission Ended")
        break
    else:
        print("Invalid command")
        continue

    battery -= 5

    if battery <= 20:
        print("Low Battery! Return to charging station")
    if x == 0 and y == 0 and battery < 100:
        print("Charging...")
        battery = 100

    print("Battery Empty!")


