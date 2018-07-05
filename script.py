import new_formulation_test_simulation as sim
import csv
	
passengers_used = []
vehicles_used = []
vehicle_speed_used = []
sim_x_size_used = []
sim_y_size_used = []
run_horizon_used = []
update_interval_used = []
dropoff_reasssignment_penalty_used = []
reassignment_penalty_used = []
waiting_penalty_used = []
pass1_distance_pen_used = []
pass2_distance_pen_used = []
rideshare_flat_penalty_used = []
rideshare_allowed = []
zetas = []

variables = [passengers_used, vehicles_used, vehicle_speed_used, sim_x_size_used, sim_y_size_used, run_horizon_used, 
			update_interval_used, dropoff_reasssignment_penalty_used, reassignment_penalty_used, waiting_penalty_used,
			pass1_distance_pen_used, pass2_distance_pen_used, rideshare_flat_penalty_used, rideshare_allowed, zetas]

run_time = []
num_served = []
empty_km_1 = []
empty_km_2 = []
total_km = []
averages = []
wait_times = []

results = [run_time, num_served, empty_km_1, empty_km_2, total_km, averages, wait_times]

passengers = 400
vehicles = 20


with open ('vs_small_new.csv','a') as file:
	writer = csv.writer(file)
	temp = ['passengers', 'vehicles', 'vehicle speed', 'x-dim', 'y-dim', 'time horizon', 'update interval', 'immediate pickup penalty', 
	'reassignment penalty', 'waiting penalty', 'pass1 penalty', 'pass2 penalty', 'rideshare flat penalty', 'zeta', 'rideshares', 'run time', 
	'number served', 'empty km 1', 'empty km 2', 'total km', 'average wait', 'wait times']
	writer.writerow(temp)

while passengers <= 500:
	while vehicles <= 30:
		for rideshare_used in range(2):
			number_of_passengers = passengers
			number_of_vehicles = vehicles
			vehicle_speed = 60. #kmh 55 default
			x_size = 10. #km
			y_size = 10. #km
			run_horizon = 3. #hours
			update_interval = 10. #seconds
			dropoff_reasssignment_penalty = 1 #km
			reassignment_penalty = 1. #km * seconds
			waiting_penalty = .05 #km/seconds
			pass1_distance_pen = 1.5
			pass2_distance_pen = 1.1
			rideshare_flat_penalty = 1.8 #km
			zeta = 1
			rideshare = True if rideshare_used == 1 else False

			#simulation is calculated in km and seconds
			vehicle_speed /= 3600. #kms
			run_horizon *= 3600. #s

			try:
				res = []
				time, served, empty1, empty2, total, average_waits, waits = sim.simulate_rideshare(number_of_passengers, number_of_vehicles, vehicle_speed, x_size, y_size, run_horizon, update_interval, dropoff_reasssignment_penalty, reassignment_penalty, waiting_penalty, pass1_distance_pen, pass2_distance_pen, rideshare_flat_penalty, rideshare, zeta)

				res.append(number_of_passengers)
				res.append(number_of_vehicles)
				res.append(vehicle_speed)
				res.append(x_size)
				res.append(y_size)
				res.append(run_horizon)
				res.append(update_interval)
				res.append(dropoff_reasssignment_penalty)
				res.append(reassignment_penalty)
				res.append(waiting_penalty)
				res.append(pass1_distance_pen)
				res.append(pass2_distance_pen)
				res.append(rideshare_flat_penalty)
				res.append(zeta)
				res.append(rideshare)
				res.append(time)
				res.append(served)
				res.append(empty1)
				res.append(empty2)
				res.append(total)
				res.append(average_waits)
				res.append(waits)

				with open ('vs_small_new.csv','a') as file:
					writer = csv.writer(file)
					writer.writerow(res)


			except Exception:
				print('broken')

		vehicles += 2
	passengers += 25
	vehicles = 20

# p1 = 1.
# p2 = 1.
# flat = 1.
# z = 0.7


# while p1 <= 3:
# 		while p2 <= 3:
# 			while flat <= 5:
# 				while z <= 2:
# 					number_of_passengers = 60
# 					number_of_vehicles = 25
# 					vehicle_speed = 60. #kmh 55 default
# 					x_size = 10. #km
# 					y_size = 10. #km
# 					run_horizon = 3. #hours
# 					update_interval = 10. #seconds
# 					dropoff_reasssignment_penalty = 1 #km
# 					reassignment_penalty = 1. #km * seconds
# 					waiting_penalty = .05 #km/seconds
# 					pass1_distance_pen = p1
# 					pass2_distance_pen = p2
# 					rideshare_flat_penalty = flat #km
# 					zeta = z
# 					rideshare = True

# 					#simulation is calculated in km and seconds
# 					vehicle_speed /= 3600. #kms
# 					run_horizon *= 3600. #s

# 					try:
# 						res = []
# 						time, served, empty1, empty2, total, average_waits, waits = sim.simulate_rideshare(number_of_passengers, number_of_vehicles, vehicle_speed, x_size, y_size, run_horizon, update_interval, dropoff_reasssignment_penalty, reassignment_penalty, waiting_penalty, pass1_distance_pen, pass2_distance_pen, rideshare_flat_penalty, rideshare, zeta)

# 						res.append(number_of_passengers)
# 						res.append(number_of_vehicles)
# 						res.append(vehicle_speed)
# 						res.append(x_size)
# 						res.append(y_size)
# 						res.append(run_horizon)
# 						res.append(update_interval)
# 						res.append(dropoff_reasssignment_penalty)
# 						res.append(reassignment_penalty)
# 						res.append(waiting_penalty)
# 						res.append(pass1_distance_pen)
# 						res.append(pass2_distance_pen)
# 						res.append(rideshare_flat_penalty)
# 						res.append(zeta)
# 						res.append(rideshare)
# 						res.append(time)
# 						res.append(served)
# 						res.append(empty1)
# 						res.append(empty2)
# 						res.append(total)
# 						res.append(average_waits)
# 						res.append(waits)

# 						with open ('weights.csv','a') as file:
# 							writer = csv.writer(file)
# 							writer.writerow(res)

# 					except Exception:
# 						print('broken')

# 					z += 0.2
				
# 				flat += 0.2
# 				z = 0.7
# 			p2 += 0.2
# 			flat = 1.
# 			z = 0.7
# 		p1 += 0.2
# 		p2 = 1.
# 		flat = 1.
# 		z = 0.7
				

# print(results)


print('worked')
