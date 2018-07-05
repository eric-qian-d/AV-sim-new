import cplex
import math
import random
import numpy as np
import sys
import csv


def simulate_rideshare(num_passengers, num_vehicles, vehicle_speed, x_max, y_max, time_length, time_interval, drop_off_pen, reassign_pen, wait_pen, pass1_pen = 1, pass2_pen = 1, rideshare_flat_penalty = 25, rideshare = True, zeta = 1):
	random.seed(12)

	'''
	Simulates a dynamic system for the specified length of time using the given inputs, updating information at every time interval. Returns
	the total amount of time waited
	'''

	#still technically need to implement d for passenger 1 (make sure they don't get stuck infinitely)
	def update_in_vehicle(R, R_IV, R_IV_1, R_IV_2, R_S, V_I, V_D, V_D_s, V_D_r, V_has_next):
		if len(R_IV) == 0: return 0
		total = 0
		v_done, r_done = [], []
		for vehicle in V_D:
			passenger = R[vehicle.passengers[vehicle.serving]]
			if dist_to_d(passenger, vehicle) < distance_travel:
				if vehicle.serving == 1:
					R[vehicle.passengers[0]].extra += dist_to_d(passenger, vehicle)
				total += dist_to_d(passenger, vehicle)
				vehicle.x, vehicle.y = passenger.d[0], passenger.d[1]
				v_done.append(vehicle)
				r_done.append(passenger)
				vehicle.in_vehicle.pop(vehicle.serving)
			else:
				if vehicle.serving == 1:
					R[vehicle.passengers[0]].extra += distance_travel
				total += distance_travel
				x_comp = abs(vehicle.x - passenger.d[0])/(abs(vehicle.x - passenger.d[0]) + abs(vehicle.y - passenger.d[1]))
				y_comp = 1 - x_comp
				theta = math.atan(y_comp/(x_comp + 0.001))
				dx = np.sign(passenger.d[0] - vehicle.x) * math.cos(theta) * distance_travel
				dy = np.sign(passenger.d[1] - vehicle.y) * math.sin(theta) * distance_travel
				vehicle.x += dx
				vehicle.y += dy
			for p in vehicle.passengers:
				if R[p] in R_IV:
					R[p].x, R[p].y = vehicle.x, vehicle.y

		#status updates on vehicles that just dropped off a passenger
		for vehicle in v_done:
			V_D.remove(vehicle)
			vehicle.passengers.pop(vehicle.serving)
			if vehicle in V_D_s:
				V_D_s.remove(vehicle)
				if vehicle.next is not None: #has a ride assigned for right after dropoff
					vehicle.serving = 0
					vehicle.passengers = [vehicle.next]
					vehicle.picking_up = 0
					vehicle.next = None
					V_P.add(vehicle)
					V_P_s.add(vehicle)
					V_has_next.remove(vehicle)
					vehicle.state = 'picking up'

				else: #
					V_I.add(vehicle)
					vehicle.serving = None
					vehicle.state = 'idle'
			else: #has a rideshare to serve
				V_D_r.remove(vehicle)
				V_D_s.add(vehicle)
				V_D.add(vehicle)
				vehicle.serving = 0
				vehicle.state = 'dropping off'
				passenger = R[vehicle.passengers[0]]
				if passenger in R_IV_2:
					R_IV_2.remove(passenger)
					R_IV_1.add(passenger)

		for passenger in r_done:
			R_IV.remove(passenger)
			R_S.add(passenger)
			passenger.state = 'served'
			passenger.vehicle = None
			if passenger in R_IV_1:
				R_IV_1.remove(passenger)
			else:
				R_IV_2.remove(passenger)

		return total

	def update_assigned(R, R_A, R_IV, R_IV_1, R_IV_2, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, t):
		if len(R_A) == 0: return 0, 0
		empty_km_1 = 0.
		empty_km_2 = 0.
		v_done, r_done = [], []
		for vehicle in V_P:
			passenger = R[vehicle.passengers[vehicle.picking_up]]
			if distance(passenger, vehicle) < distance_travel:
				if vehicle.serving == 1:
					R[vehicle.passengers[0]].extra += distance(passenger, vehicle)
				empty_km_1 += distance(passenger, vehicle) #diagnostic
				if vehicle in V_P_s:
					empty_km_2 += distance(passenger, vehicle)
				vehicle.x, vehicle.y = passenger.o[0], passenger.o[1]
				v_done.append(vehicle)
				r_done.append(passenger)
				vehicle.in_vehicle.append(vehicle.picking_up)
				passenger.picked_up = t
			else:
				if vehicle.serving == 1:
					R[vehicle.passengers[0]].extra += distance_travel
				empty_km_1 += distance_travel
				if vehicle in V_P_s:
					empty_km_2 += distance_travel
				x_comp = abs(vehicle.x - passenger.o[0])/(abs(vehicle.x - passenger.o[0]) + abs(vehicle.y - passenger.o[1]))
				y_comp = 1 - x_comp
				theta = math.atan(y_comp/(x_comp + 0.001))
				dx = np.sign(passenger.o[0] - vehicle.x) * math.cos(theta) * distance_travel
				dy = np.sign(passenger.o[1] - vehicle.y) * math.sin(theta) * distance_travel
				vehicle.x += dx
				vehicle.y += dy
			for p in vehicle.passengers:
				if R[p] in R_IV:
					R[p].x, R[p].y = vehicle.x, vehicle.y

		for vehicle in v_done:
			V_P.remove(vehicle)
			V_D.add(vehicle)
			vehicle.state = 'dropping off'
			vehicle.picking_up = None
			if vehicle in V_P_s:
				V_D_s.add(vehicle)
				V_P_s.remove(vehicle)
			else:
				V_D_r.add(vehicle)
				V_P_r.remove(vehicle)

		for passenger in r_done:
			R_A.remove(passenger)
			R_IV.add(passenger)
			R_prime.remove(passenger)
			passenger.state = 'in vehicle'
			if len(V[passenger.vehicle].passengers) == 1 or passenger.num == V[passenger.vehicle].passengers[0]: #could also use if V[passenger.vehicle] in V_D_s
				R_IV_1.add(passenger)
			else:
				R_IV_2.add(passenger)

		return empty_km_1, empty_km_2

	def update_unassigned(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next):
		if len(R_U) < 1: return
		solve_R_greaterthan_V(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next) if len(R_prime) + len(R_IV) > 2 * len(V_D) + 2 * len(V_P_r) + len(V_P_s) + len(V_I) else solve_R_lessthan_V(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next)


	def solve_R_lessthan_V(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next):
		#Initialize variables
		R_joined = R_prime.union(R_IV)

		d, y = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		x_prev, x_prime_prev = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		d11, d12, d21, rideshare_pen = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)]
		p, q = [0] * len(V), [0] * len(V)
		b, in_car_adjustment, extra = [0] * len(R), [0] * len(R), [0] * len(R)

		for j in range(num_vehicles):

			p[j] = 1 if len(V[j].in_vehicle) > 0 else 0 
			q[j] = 1 if V[j] in V_P else 0

			for i in range(num_passengers):
				y[i][j] = 0
				if R[i].vehicle == j:
					y[i][j] = 1

				if V[j] in V_D:
					d[i][j] = dist_to_d(R[V[j].passengers[V[j].serving]], V[j]) + point_dist(R[V[j].passengers[V[j].serving]].d, R[i].o)
				else:
					d[i][j] = distance(R[i], V[j])

		for i in range(num_passengers):
			b[i] = R[i].reassigned
			in_car_adjustment[i] = 0 if R[i] in R_IV else 1
			extra[i] = R[i].extra

		for i in range(num_passengers):
			for j in range(num_vehicles):
				if V[j] in V_D and R[i] in R_prime:
					vehicle = V[j]
					current_passenger = R[V[j].passengers[0]]
					considered_passenger = R[i]
					d11[i][j] = point_dist((vehicle.x, vehicle.y), considered_passenger.o) + point_dist(considered_passenger.o, current_passenger.d) - dist_to_d(current_passenger, vehicle)
					d12[i][j] = point_dist(considered_passenger.o, current_passenger.d) + point_dist(current_passenger.d, considered_passenger.d) - point_dist(considered_passenger.o, considered_passenger.d)
					d21[i][j] = point_dist((vehicle.x, vehicle.y), considered_passenger.o) + point_dist(considered_passenger.o, considered_passenger.d) + point_dist(considered_passenger.d, current_passenger.d) - dist_to_d(current_passenger, vehicle) + extra[i]				
					rideshare_pen[i][j] = [psi1 * (d11[i][j] + distance(considered_passenger, vehicle)) + psi2 * d12[i][j] - zeta * point_dist(considered_passenger.o, current_passenger.d) , 0] if psi1 * d11[i][j] + psi2 * d12[i][j] - zeta * point_dist(considered_passenger.o, current_passenger.d) < psi1 * d21[i][j] - zeta * point_dist(considered_passenger.o, considered_passenger.d) else [psi1 * (d21[i][j] + distance(considered_passenger, vehicle)) - zeta * point_dist(considered_passenger.o, considered_passenger.d), 1]
				else:
					d11[i][j] = 0
					d12[i][j] = 0
					d21[i][j] = 0
					rideshare_pen[i][j] = [0, None]

		
		for passenger in R_IV:
			v = passenger.vehicle
			vehicle = V[v]
			if len(vehicle.passengers) == 1 or passenger.num == vehicle.passengers[0]:
				x_prev[passenger.num][v] = 1
			else:
				x_prime_prev[passenger.num][v] = 1

		seats = min(len(R_joined), 2 * len(V_D) + 2 * len(V_P_r) + len(V_P_s) + len(V_I))
		V_joined = V_P_r.union(V_D)

		#Create new model for |R'| < |V'|
		problem = cplex.Cplex()
    		problem.objective.set_sense(problem.objective.sense.minimize)

		#Set variables to optimize
		obj = []
		lb = []
		ub = []
		names = []
		variable_types = []

		for i in R_joined:
			for j in V_prime:
				names.append('x({0},{1})'.format(i.num, j.num))
				obj.append((d[i.num][j.num] + phi * p[j.num] + delta * q[j.num] * (1 - y[i.num][j.num])) * in_car_adjustment[i.num])
				lb.append(0)
				ub.append(1)
				variable_types.append("C")
				
				names.append('x_prime({0},{1})'.format(i.num, j.num))
				obj.append((rideshare_pen[i.num][j.num][0] + delta * q[j.num] * (1 - y[i.num][j.num]) + rideshare_flat_penalty) * in_car_adjustment[i.num])
				lb.append(0)
				ub.append(1)
				variable_types.append("C")

		#Set constraints
		constraint_names = []
		constraints = []
		constraint_rhs = []
		constraint_sense = []

		for j in V_prime:
			#makes sure that there's already passenger 1 before rideshare is assigned
			new_rideshare_constraint = [[], []]
			for i in R_joined:
				new_rideshare_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				new_rideshare_constraint[1].append(1)
				
			constraint_names.append('initial_rider_constraint_{0}'.format(j.num))
			constraints.append(new_rideshare_constraint)
			constraint_rhs.append(p[j.num])
			constraint_sense.append('L')	

			#caps single rides at 1
			single_ride_cap_constraint = [[], []]
			for i in R_prime:
				single_ride_cap_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				single_ride_cap_constraint[1].append(1)
				
			constraint_names.append('single_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(single_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')

			#caps shared rides at 1
			shared_ride_cap_constraint = [[], []]
			for i in R_joined:
				shared_ride_cap_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				shared_ride_cap_constraint[1].append(1)
				
			constraint_names.append('shared_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(shared_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')
			
		#makes sure that every passenger is assigned to exactly 1 vehicle since |R| < |V|
		for i in R_joined:
			passenger_assigned_constraint = [[], []]
			for j in V_prime:
				passenger_assigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				passenger_assigned_constraint[1].extend([1,1])
				
			constraint_names.append('passenger_{0}_assigned_constraint'.format(i.num))
			constraints.append(passenger_assigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('E')
			
		#only one reassignment
		for i in R_joined:
			for j in V_prime:
				one_standard_reassignment_constraint = [[], []]
				one_standard_reassignment_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
				one_standard_reassignment_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
			
				constraint_names.append('passenger_{0}_vehicle_{1}_one_reassignment'.format(i.num, j.num))
				constraints.append(one_standard_reassignment_constraint)
				constraint_rhs.append(b[i.num] * y[i.num][j.num])
				constraint_sense.append('G')

		#2 assignments max per car
		for j in V_prime:
			vehicle_max_2_assigned_constraint = [[], []]
			for i in R_joined:
				vehicle_max_2_assigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				vehicle_max_2_assigned_constraint[1].extend([1,1])

			constraint_names.append('vehicle_{0}_max_2_assigned_constraint'.format(j.num))
			constraints.append(vehicle_max_2_assigned_constraint)
			constraint_rhs.append(2)
			constraint_sense.append('L')

		#makes sure we don't kick passengers out of cars they're already in
		for i in R_IV_1:
			for j in V_joined:
				no_kick_out_constraint_x = [[], []]
				no_kick_out_constraint_x[0].append('x({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x[1].append(1)
				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x)
				constraint_rhs.append(x_prev[i.num][j.num])
				constraint_sense.append('E')

		for i in R_IV_2:
			for j in V_joined:
				no_kick_out_constraint_x_prime = [[], []]
				no_kick_out_constraint_x_prime[0].append('x_prime({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x_prime[1].append(1)
				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x_prime'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x_prime)
				constraint_rhs.append(x_prime_prev[i.num][j.num])
				constraint_sense.append('E')


		problem.variables.add(obj = obj, lb = lb, ub = ub, names = names, types = variable_types)
		problem.linear_constraints.add(lin_expr = constraints, senses = constraint_sense, rhs = constraint_rhs, names = constraint_names)

		problem.set_log_stream(None)
		problem.set_results_stream(None)
		problem.set_error_stream(None)
		problem.solve()
		
		values = problem.solution.get_values()
		
		#updates sets of assigned passengers and vehicles for the next time step
		#reconfigures vehicle and passenger info due to reassignments
		for ind in range(len(names)):
			var = names[ind]
			first_start = var.find('(') + 1 #13
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if passenger in R_prime:
				if values[ind] == 1:
					if passenger in R_A:  
						if passenger.vehicle != v:
							passenger.reassigned = 1
							if V[passenger.vehicle] in V_P_s: #reassigned passenger was reassigned from a single ride (V_P 0)
								V_I.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V_P_s.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'idle'
								V[passenger.vehicle].passengers = []
								V[passenger.vehicle].serving = None
							elif V[passenger.vehicle] in V_P_r: #reassigned passenger was reassigned from the second passenger of a shared ride (V_P 1)
								V_D.add(V[passenger.vehicle])
								V_D_s.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V_P_r.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'dropping off'
								V[passenger.vehicle].passengers.pop(1)
								V[passenger.vehicle].serving = 0
							else: #reassigned passenger must've come from a vehicle in V_D
								V[passenger.vehicle].next = None

		#takes care of all passenger assignments
		for ind in range(len(names)): 
			var = names[ind]
			first_start = var.find('(') + 1
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if values[ind] == 1: 
				if passenger in R_prime and not(p in vehicle.passengers or p == vehicle.next):
					R_A.add(passenger)
					passenger.state = 'assigned'
					passenger.vehicle = v

					if passenger in R_U:
						R_U.remove(passenger)

					if 'x(' in var: #case where a single ride is assigned/reassigned
						if vehicle in V_I: #passenger will be picked up immediately next
							V_I.remove(vehicle)
							V_P.add(vehicle)
							V_P_s.add(vehicle)
							vehicle.passengers = [p]
							vehicle.picking_up = 0
							vehicle.serving = 0
							vehicle.state = 'picking up'

						elif vehicle in V_P_s:
							vehicle.passengers = [p]
							vehicle.picking_up = 0
							vehicle.serving = 0

						elif vehicle in V_P_r:
							vehicle.passengers[1] = p
							vehicle.picking_up = 1
							vehicle.serving = 0 if rideshare_pen[p][v][1] == 0 else 1					
														
						else: #passenger will be picked up after the next passenger is dropped off
							vehicle.next = p
							V_has_next.add(vehicle)
						
					else:
						if vehicle in V_P_r: #when a rideshare gets reassigned to another rideshare
							vehicle.passengers[1] = p

						else: #when a new rideshare assigned
							vehicle.passengers.append(p)
							V_P_r.add(vehicle)
							V_D_s.remove(vehicle)
							V_D.remove(vehicle)

						V_P.add(vehicle)
						vehicle.state = 'picking up'
						vehicle.picking_up = 1
						vehicle.serving = 0 if rideshare_pen[p][v][1] == 0 else 1


	def solve_R_greaterthan_V(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next):
		#Initialize variables
		R_joined = R_prime.union(R_IV)

		d, y = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		x_prev, x_prime_prev = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		d11, d12, d21, rideshare_pen = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)]
		p, q = [0] * len(V), [0] * len(V)
		b, in_car_adjustment, wait, extra = [0] * len(R), [0] * len(R), [0] * len(R), [0] * len(R)

		for j in range(num_vehicles):
			p[j] = 1 if len(V[j].in_vehicle) > 0 else 0 
			q[j] = 1 if V[j] in V_P else 0

			for i in range(num_passengers):
				y[i][j] = 0
				if R[i].vehicle == j:
					y[i][j] = 1

				if V[j] in V_D:
					d[i][j] = dist_to_d(R[V[j].passengers[V[j].serving]], V[j]) + point_dist(R[V[j].passengers[V[j].serving]].d, R[i].o)
				else:
					d[i][j] = distance(R[i], V[j])

		for i in range(num_passengers):
			b[i] = R[i].reassigned
			in_car_adjustment[i] = 0 if R[i] in R_IV else 1
			extra[i] = R[i].extra
			wait[i] = R[i].wait

		for i in range(num_passengers):

			for j in range(num_vehicles):
				if V[j] in V_D and R[i] in R_prime:
					vehicle = V[j]
					current_passenger = R[V[j].passengers[0]]
					considered_passenger = R[i]
					d11[i][j] = point_dist((vehicle.x, vehicle.y), considered_passenger.o) + point_dist(considered_passenger.o, current_passenger.d) - dist_to_d(current_passenger, vehicle)
					d12[i][j] = point_dist(considered_passenger.o, current_passenger.d) + point_dist(current_passenger.d, considered_passenger.d) - point_dist(considered_passenger.o, considered_passenger.d)
					d21[i][j] = point_dist((vehicle.x, vehicle.y), considered_passenger.o) + point_dist(considered_passenger.o, considered_passenger.d) + point_dist(considered_passenger.d, current_passenger.d) - dist_to_d(current_passenger, vehicle) + extra[i]
					rideshare_pen[i][j] = [psi1 * (d11[i][j] + distance(considered_passenger, vehicle)) + psi2 * d12[i][j] - zeta * point_dist(considered_passenger.o, current_passenger.d) , 0] if psi1 * d11[i][j] + psi2 * d12[i][j] - zeta * point_dist(considered_passenger.o, current_passenger.d) < psi1 * d21[i][j] - zeta * point_dist(considered_passenger.o, considered_passenger.d) else [psi1 * (d21[i][j] + distance(considered_passenger, vehicle)) - zeta * point_dist(considered_passenger.o, considered_passenger.d), 1]
				else:
					d11[i][j] = 0
					d12[i][j] = 0
					d21[i][j] = 0
					rideshare_pen[i][j] = [0, None]

		for passenger in R_IV:
			v = passenger.vehicle
			vehicle = V[v]
			if len(vehicle.passengers) == 1 or passenger.num == vehicle.passengers[0]:
				x_prev[passenger.num][v] = 1
			else:
				x_prime_prev[passenger.num][v] = 1

		seats = min(len(R_joined), 2 * len(V_D) + 2 * len(V_P_r) + len(V_P_s) + len(V_I))
		V_joined = V_P_r.union(V_D)

		#Create new model for |R'| > |V'|
		problem = cplex.Cplex()
		problem.objective.set_sense(problem.objective.sense.minimize)

		#Set variables to optimize
		obj = []
		lb = []
		ub = []
		names = []
		variable_types = []

		for i in R_joined:
			for j in V_prime:
				names.append('x({0},{1})'.format(i.num, j.num))
				obj.append((d[i.num][j.num] + phi * p[j.num] + delta * q[j.num] * (1 - y[i.num][j.num]) - gamma * wait[i.num]) * in_car_adjustment[i.num])
				lb.append(0)
				ub.append(1)
				variable_types.append("C")
				
				names.append('x_prime({0},{1})'.format(i.num, j.num))
				obj.append((rideshare_pen[i.num][j.num][0] + delta * q[j.num] * (1 - y[i.num][j.num]) + rideshare_flat_penalty - gamma * wait[i.num]) * in_car_adjustment[i.num])
				lb.append(0)
				ub.append(1)
				variable_types.append("C")

		#Set constraints
		constraint_names = []
		constraints = []
		constraint_rhs = []
		constraint_sense = []

		for j in V_prime:
			#makes sure that there's already passenger 1 before rideshare is assigned
			new_rideshare_constraint = [[], []]
			for i in R_joined:
				new_rideshare_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				new_rideshare_constraint[1].append(1)
				
			constraint_names.append('initial_rider_constraint_{0}'.format(j.num))
			constraints.append(new_rideshare_constraint)
			constraint_rhs.append(p[j.num])
			constraint_sense.append('L')	

			#caps single rides at 1
			single_ride_cap_constraint = [[], []]
			for i in R_prime:
				single_ride_cap_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				single_ride_cap_constraint[1].append(1)
				
			constraint_names.append('single_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(single_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')

			#caps shared rides at 1
			shared_ride_cap_constraint = [[], []]
			for i in R_joined:
				shared_ride_cap_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				shared_ride_cap_constraint[1].append(1)
				
			constraint_names.append('shared_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(shared_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')
			
		#makes sure that every passenger is assigned to at most 1 vehicle since |V| < |R|
		for i in R_joined:
			passenger_assigned_constraint = [[], []]
			for j in V_prime:
				passenger_assigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				passenger_assigned_constraint[1].extend([1,1])
				
			constraint_names.append('passenger_{0}_assigned_constraint'.format(i.num))
			constraints.append(passenger_assigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')

		#only one reassignment
		for i in R_joined:
			for j in V_prime:
				one_standard_reassignment_constraint = [[], []]
				one_standard_reassignment_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
				one_standard_reassignment_constraint[0].append('x_prime({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
			
				constraint_names.append('passenger_{0}_vehicle_{1}_one_reassignment'.format(i.num, j.num))
				constraints.append(one_standard_reassignment_constraint)
				constraint_rhs.append(b[i.num] * y[i.num][j.num])
				constraint_sense.append('G')

		#max 2 assignments
		for j in V_prime:
			vehicle_max_2_assigned_constraint = [[], []]
			for i in R_joined:
				vehicle_max_2_assigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				vehicle_max_2_assigned_constraint[1].extend([1,1])

			constraint_names.append('vehicle_{0}_max_2_assigned_constraint'.format(j.num))
			constraints.append(vehicle_max_2_assigned_constraint)
			constraint_rhs.append(2)
			constraint_sense.append('L')

		#makes sure we don't kick passengers out of cars they're already in
		for i in R_IV_1:
			for j in V_joined:
				no_kick_out_constraint_x = [[], []]
				no_kick_out_constraint_x[0].append('x({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x[1].append(1)
				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x)
				constraint_rhs.append(x_prev[i.num][j.num])
				constraint_sense.append('E')

		for i in R_IV_2:
			for j in V_joined:
				no_kick_out_constraint_x_prime = [[], []]
				no_kick_out_constraint_x_prime[0].append('x_prime({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x_prime[1].append(1)
				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x_prime'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x_prime)
				constraint_rhs.append(x_prime_prev[i.num][j.num])
				constraint_sense.append('E')

		#prevents going from assigned to unassigned
		for i in R_A:
			no_become_unassigned_constraint = [[],[]]
			for j in V_prime:
				no_become_unassigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				no_become_unassigned_constraint[1].extend([1,1])

			constraint_names.append('passenger_{0}_no_become_unassigned'.format(i.num))
			constraints.append(no_become_unassigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('E')
			
		#makes sure that all possible open seats are used to assign passengers
		max_seats_constraint = [[], []]
		for i in R_joined:
			for j in V_prime:
				max_seats_constraint[0].extend(['x({0},{1})'.format(i.num, j.num),'x_prime({0},{1})'.format(i.num, j.num)])
				max_seats_constraint[1].extend([1,1])

		constraint_names.append('max_seats_used_constraint')
		constraints.append(max_seats_constraint)
		constraint_rhs.append(seats)
		constraint_sense.append('E')

		problem.variables.add(obj = obj, lb = lb, ub = ub, names = names, types = variable_types)
		problem.linear_constraints.add(lin_expr = constraints, senses = constraint_sense, rhs = constraint_rhs, names = constraint_names)

		problem.set_log_stream(None)
		problem.set_error_stream(None)
		problem.set_results_stream(None)
		problem.solve()
		
		values = problem.solution.get_values()
		
		#updates sets of assigned passengers and vehicles for the next time step
		#reconfigures vehicle and passenger info due to reassignments
		for ind in range(len(names)):
			var = names[ind]
			first_start = var.find('(') + 1 #13
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if passenger in R_prime:
				if values[ind] == 1:
					if passenger in R_A:  
						if passenger.vehicle != v:
							passenger.reassigned = 1
							if V[passenger.vehicle] in V_P_s: #reassigned passenger was reassigned from a single ride (V_P 0)
								V_I.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V_P_s.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'idle'
								V[passenger.vehicle].passengers = []
								V[passenger.vehicle].serving = None
							elif V[passenger.vehicle] in V_P_r: #reassigned passenger was reassigned from the second passenger of a shared ride (V_P 1)
								V_D.add(V[passenger.vehicle])
								V_D_s.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V_P_r.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'dropping off'
								V[passenger.vehicle].passengers.pop(1)
								V[passenger.vehicle].serving = 0
							else: #reassigned passenger must've come from a vehicle in V_D
								V[passenger.vehicle].next = None

		#takes care of all passenger assignments
		for ind in range(len(names)): 
			var = names[ind]
			first_start = var.find('(') + 1
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if values[ind] == 1: 
				if passenger in R_prime and not(p in vehicle.passengers or p == vehicle.next):
				 
					R_A.add(passenger)
					passenger.state = 'assigned'
					passenger.vehicle = v

					if passenger in R_U:
						R_U.remove(passenger)

					if 'x(' in var: #case where a single ride is assigned/reassigned
						if vehicle in V_I: #passenger will be picked up immediately next
							V_I.remove(vehicle)
							V_P.add(vehicle)
							V_P_s.add(vehicle)
							vehicle.passengers = [p]
							vehicle.picking_up = 0
							vehicle.serving = 0
							vehicle.state = 'picking up'

						elif vehicle in V_P_s:
							vehicle.passengers = [p]
							vehicle.picking_up = 0
							vehicle.serving = 0

						elif vehicle in V_P_r:
							vehicle.passengers[1] = p
							vehicle.picking_up = 1
							vehicle.serving = 0 if rideshare_pen[p][v][1] == 0 else 1					
														
						else: #passenger will be picked up after the next passenger is dropped off
							vehicle.next = p
							V_has_next.add(vehicle)
					else:
						if vehicle in V_P_r: #when a rideshare gets reassigned to another rideshare
							vehicle.passengers[1] = p

						else: #when a new rideshare assigned
							vehicle.passengers.append(p)
							V_P_r.add(vehicle)
							V_D_s.remove(vehicle)
							V_D.remove(vehicle)

						V_P.add(vehicle)
						vehicle.state = 'picking up'
						vehicle.picking_up = 1
						vehicle.serving = 0 if rideshare_pen[p][v][1] == 0 else 1
		
		for passenger in R_U:
			passenger.wait += time_interval

		
#<---------------------------------------------------------------------------------------------------------------------------------------------------

	def update_in_vehicle_s(R, R_IV, R_S, V_I, V_D): #need to consider order of dropoff for rideshares; FIX!
		if len(R_IV) == 0: return 0
		total = 0
		v_done, r_done = [], []
		for vehicle in V_D:
			passenger = R[vehicle.passengers[vehicle.serving]]
			if dist_to_d(passenger, vehicle) < distance_travel:
				total += dist_to_d(passenger, vehicle)
				vehicle.x, vehicle.y = passenger.d[0], passenger.d[1]
				v_done.append(vehicle)
				r_done.append(passenger)
			else:
				total += distance_travel
				x_comp = abs(vehicle.x - passenger.d[0])/(abs(vehicle.x - passenger.d[0]) + abs(vehicle.y - passenger.d[1]))
				y_comp = 1 - x_comp
				theta = math.atan(y_comp/(x_comp + 0.001))
				dx = np.sign(passenger.d[0] - vehicle.x) * math.cos(theta) * distance_travel
				dy = np.sign(passenger.d[1] - vehicle.y) * math.sin(theta) * distance_travel
				vehicle.x += dx
				vehicle.y += dy
			for p in vehicle.passengers:
				if R[p] in R_IV:
					R[p].x, R[p].y = vehicle.x, vehicle.y

		for vehicle in v_done:
			V_D.remove(vehicle)
			vehicle.passengers.pop(vehicle.serving)
			
			if vehicle.next is not None:
				vehicle.serving = 0
				vehicle.passengers = [vehicle.next]
				vehicle.picking_up = 0
				vehicle.next = None
				V_P.add(vehicle)
				vehicle.state = 'drop off'
			else:
				V_I.add(vehicle)
				vehicle.serving = None
				vehicle.state = 'idle'

		for passenger in r_done:
			R_IV.remove(passenger)
			R_S.add(passenger)
			passenger.state = 'served'
			passenger.vehicle = None

		return total

	def update_assigned_s(R, R_A, R_IV, V_P, V_D, t):
		if len(R_A) == 0: return 0, 0

		empty_km_1 = 0
		empty_km_2 = 0
		v_done, r_done = [], []
		for vehicle in V_P:
			passenger = R[vehicle.passengers[vehicle.picking_up]]
			if distance(passenger, vehicle) < distance_travel:
				empty_km_1 += distance(passenger, vehicle) #diagnostic
				empty_km_2 += distance(passenger, vehicle)
				vehicle.x, vehicle.y = passenger.o[0], passenger.o[1]
				v_done.append(vehicle)
				r_done.append(passenger)
				passenger.picked_up = t
			else:
				empty_km_1 += distance_travel #diagnostic
				empty_km_2 += distance_travel
				x_comp = abs(vehicle.x - passenger.o[0])/(abs(vehicle.x - passenger.o[0]) + abs(vehicle.y - passenger.o[1]))
				y_comp = 1 - x_comp
				theta = math.atan(y_comp/(x_comp + 0.001))
				dx = np.sign(passenger.o[0] - vehicle.x) * math.cos(theta) * distance_travel
				dy = np.sign(passenger.o[1] - vehicle.y) * math.sin(theta) * distance_travel
				vehicle.x += dx
				vehicle.y += dy

		for vehicle in v_done:
			V_P.remove(vehicle)
			V_D.add(vehicle)
			vehicle.state = 'drop off'
			vehicle.picking_up = None

		for passenger in r_done:
			R_A.remove(passenger)
			R_IV.add(passenger)
			R_prime.remove(passenger)
			passenger.state = 'in vehicle'

		return empty_km_1, empty_km_2

	def update_unassigned_s(R, R_A, R_IV, R_prime, V, V_P, V_prime):
		if len(R_U) < 1: return
		solve_R_greaterthan_V_s(R, R_A, R_IV, R_prime, V, V_P, V_prime) if len(R_prime) >  len(V_D) + len(V_P) + len(V_I) else solve_R_lessthan_V_s(R, R_A, R_IV, R_prime, V, V_P, V_prime)


	def solve_R_lessthan_V_s(R, R_A, R_IV, R_prime, V, V_P, V_prime):
		#Initialize variables
		d, y = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		x_prev = [[0 for j in range(num_vehicles)] for i in range(num_passengers)]
		p, q = [0] * len(V), [0] * len(V)
		b, c = [0] * len(R), [0] * len(R)

		for j in range(num_vehicles):
			p[j] = 1 if V[j] in V_D else 0 
			q[j] = 1 if V[j] in V_P else 0

			for i in range(num_passengers):
				y[i][j] = 0
				if R[i].vehicle == j:
					y[i][j] = 1
				if V[j] in V_D:
					d[i][j] = dist_to_d(R[V[j].passengers[V[j].serving]], V[j]) + point_dist(R[V[j].passengers[V[j].serving]].d, R[i].o)
				else:
					d[i][j] = distance(R[i], V[j])

		for i in range(num_passengers):
			b[i] = R[i].reassigned
			c[i] = 0 if R[i] in R_IV else 1

		for passenger in R_IV:
			v = passenger.vehicle
			vehicle = V[v]
			if passenger.num == vehicle.passengers[0]:
				x_prev[passenger.num][v] = 1

		R_joined = R_prime.union(R_IV)
		V_joined = set().union(V_D)

		#Create new model for |R'| < |V'|
		problem = cplex.Cplex()
		problem.objective.set_sense(problem.objective.sense.minimize)

		#Set variables to optimize
		obj = []
		lb = []
		ub = []
		names = []
		variable_types = []

		for i in R_joined:
			for j in V_prime:
				names.append('x({0},{1})'.format(i.num, j.num))
				obj.append(c[i.num]*(d[i.num][j.num] + phi * p[j.num] + delta * q[j.num] * (1 - y[i.num][j.num])))
				lb.append(0)
				ub.append(1)
				variable_types.append("C")

		#Set constraints
		constraint_names = []
		constraints = []
		constraint_rhs = []
		constraint_sense = []

		for j in V_prime:
			#caps single rides at 1
			single_ride_cap_constraint = [[], []]
			for i in R_prime:
				single_ride_cap_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				single_ride_cap_constraint[1].append(1)
				
			constraint_names.append('single_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(single_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')
			
		#makes sure that every passenger is assigned to exactly 1 vehicle since |R| < |V|
		for i in R_joined:
			passenger_assigned_constraint = [[], []]
			for j in V_prime:
				passenger_assigned_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				passenger_assigned_constraint[1].append(1)
				
			constraint_names.append('passenger_{0}_assigned_constraint'.format(i.num))
			constraints.append(passenger_assigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('E')
			
		#only one reassignment
		for i in R_joined:
			for j in V_prime:
				one_standard_reassignment_constraint = [[], []]
				one_standard_reassignment_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
			
				constraint_names.append('passenger_{0}_vehicle_{1}_one_reassignment'.format(i.num, j.num))
				constraints.append(one_standard_reassignment_constraint)
				constraint_rhs.append(b[i.num] * y[i.num][j.num])
				constraint_sense.append('G')

		#makes sure we don't kick passengers out of cars they're already in
		#may need to generate a separate list of xij and x'ij lists to make sure that this is guaranteed to not change; working off comparisons with y list may not work

		for i in R_IV:
			for j in V_joined:
				no_kick_out_constraint_x = [[], []]
				no_kick_out_constraint_x[0].append('x({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x[1].append(1)

				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x)
				constraint_rhs.append(x_prev[i.num][j.num])
				constraint_sense.append('E')
				# print(no_kick_out_constraint_x, i.num, j.num, x_prev[i.num][j.num])


		problem.variables.add(obj = obj, lb = lb, ub = ub, names = names, types = variable_types)
		problem.linear_constraints.add(lin_expr = constraints, senses = constraint_sense, rhs = constraint_rhs, names = constraint_names)

		problem.set_log_stream(None)
		problem.set_results_stream(None)
		problem.set_error_stream(None)
		problem.solve()
		
		values = problem.solution.get_values()

		#reconfigures vehicle and passenger info due to reassignments
		for ind in range(len(names)):
			var = names[ind]
			first_start = var.find('(') + 1
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if passenger in R_prime:
				if values[ind] == 1:
					if passenger in R_A:  
						if passenger.vehicle != v:
							passenger.reassigned = 1
							if V[passenger.vehicle] in V_P:
								#reassigned passenger was reassigned from being picked up, and is being picked up by a different vehicle
								V_I.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'idle'
								V[passenger.vehicle].passengers = []
								V[passenger.vehicle].serving = None
							else: #reassigned passenger must've come from a vehicle in V_D
								V[passenger.vehicle].next = None

		#takes care of all passenger assignments
		for ind in range(len(names)): 
			var = names[ind]
			first_start = var.find('(') + 1
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if values[ind] == 1: 
				if passenger in R_prime and not(p in vehicle.passengers or p == vehicle.next):
					R_A.add(passenger)
					passenger.state = 'assigned'
					passenger.vehicle = v

					if passenger in R_U:
						R_U.remove(passenger)

					if vehicle in V_I: #passenger will be picked up immediately next
						V_I.remove(vehicle)
						V_P.add(vehicle)
						V_P_s.add(vehicle)
						vehicle.passengers = [p]
						vehicle.picking_up = 0
						vehicle.serving = 0
						vehicle.state = 'picking up'

					elif vehicle in V_P:
						vehicle.passengers = [p]
						vehicle.picking_up = 0
						vehicle.serving = 0			
													
					else: #passenger will be picked up after the next passenger is dropped off
						vehicle.next = p
						V_has_next.add(vehicle)


	def solve_R_greaterthan_V_s(R, R_A, R_IV, R_prime, V, V_P, V_prime):
		#Initialize variables
		d, y = [[0 for j in range(num_vehicles)] for i in range(num_passengers)], [[0 for j in range(num_vehicles)] for i in range(num_passengers)] 
		x_prev = [[0 for j in range(num_vehicles)] for i in range(num_passengers)]
		p, q = [0] * len(V), [0] * len(V)
		b, c, w = [0] * len(R), [0] * len(R), [0] * len(R)

		for j in range(num_vehicles):
			p[j] = 1 if V[j] in V_D else 0 
			q[j] = 1 if V[j] in V_P else 0

			for i in range(num_passengers):
				y[i][j] = 0
				if R[i].vehicle == j:
					y[i][j] = 1
				if V[j] in V_D:
					d[i][j] = dist_to_d(R[V[j].passengers[V[j].serving]], V[j]) + point_dist(R[V[j].passengers[V[j].serving]].d, R[i].o)
				else:
					d[i][j] = distance(R[i], V[j])

		for i in range(num_passengers):
			b[i] = R[i].reassigned
			c[i] = 0 if R[i] in R_IV else 1
			w[i] = R[i].wait


		for passenger in R_IV:
			v = passenger.vehicle
			vehicle = V[v]
			if passenger.num == vehicle.passengers[0]:
				x_prev[passenger.num][v] = 1

		seats = min(len(R_prime), 2 * len(V_D) + len(V_P) + len(V_I))
		R_joined = R_prime.union(R_IV)
		V_joined = set().union(V_D)

		#Create new model for |R'| > |V'|
		problem = cplex.Cplex()
		problem.objective.set_sense(problem.objective.sense.minimize)

		#Set variables to optimize
		obj = []
		lb = []
		ub = []
		names = []
		variable_types = []

		for i in R_joined:
			for j in V_prime:
				names.append('x({0},{1})'.format(i.num, j.num))
				obj.append(c[i.num]*(d[i.num][j.num] + phi * p[j.num] + delta * q[j.num] * (1 - y[i.num][j.num]) - gamma * w[i.num]))
				lb.append(0)
				ub.append(1)
				variable_types.append("C")

		#Set constraints
		constraint_names = []
		constraints = []
		constraint_rhs = []
		constraint_sense = []

		for j in V_prime:
			#caps single rides at 1
			single_ride_cap_constraint = [[], []]
			for i in R_prime:
				single_ride_cap_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				single_ride_cap_constraint[1].append(1)
				
			constraint_names.append('single_ride_cap_constraint_{0}'.format(j.num))
			constraints.append(single_ride_cap_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')
			
		#makes sure that every passenger is assigned to at most 1 vehicle since |R| > |V|
		for i in R_joined:
			passenger_assigned_constraint = [[], []]
			for j in V_prime:
				passenger_assigned_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				passenger_assigned_constraint[1].append(1)
				
			constraint_names.append('passenger_{0}_assigned_constraint'.format(i.num))
			constraints.append(passenger_assigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('L')
			
		#only one reassignment
		for i in R_joined:
			for j in V_prime:
				one_standard_reassignment_constraint = [[], []]
				one_standard_reassignment_constraint[0].append('x({0},{1})'.format(i.num, j.num))
				one_standard_reassignment_constraint[1].append(b[i.num])
			
				constraint_names.append('passenger_{0}_vehicle_{1}_one_reassignment'.format(i.num, j.num))
				constraints.append(one_standard_reassignment_constraint)
				constraint_rhs.append(b[i.num] * y[i.num][j.num])
				constraint_sense.append('G')

		#max 2 assignments
		for j in V_prime:
			vehicle_max_2_assigned_constraint = [[], []]
			for i in R_joined:
				vehicle_max_2_assigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num)])
				vehicle_max_2_assigned_constraint[1].extend([1])

			constraint_names.append('vehicle_{0}_max_2_assigned_constraint'.format(j.num))
			constraints.append(vehicle_max_2_assigned_constraint)
			constraint_rhs.append(2)
			constraint_sense.append('L')

		#prevents going from assigned to unassigned
		for i in R_A:
			no_become_unassigned_constraint = [[],[]]
			for j in V_prime:
				no_become_unassigned_constraint[0].extend(['x({0},{1})'.format(i.num, j.num)])
				no_become_unassigned_constraint[1].extend([1])

			constraint_names.append('passenger_{0}_no_become_unassigned'.format(i.num))
			constraints.append(no_become_unassigned_constraint)
			constraint_rhs.append(1)
			constraint_sense.append('E')

		#makes sure we don't kick passengers out of cars they're already in
		#may need to generate a separate list of xij and x'ij lists to make sure that this is guaranteed to not change; working off comparisons with y list may not work

		for i in R_IV:
			for j in V_joined:
				no_kick_out_constraint_x = [[], []]
				no_kick_out_constraint_x[0].append('x({0},{1})'.format(i.num, j.num))
				no_kick_out_constraint_x[1].append(1)

				constraint_names.append('passenger_{0}_in_vehicle_{1}_no_swap_x'.format(i.num, j.num))
				constraints.append(no_kick_out_constraint_x)
				constraint_rhs.append(x_prev[i.num][j.num])
				constraint_sense.append('E')

		problem.variables.add(obj = obj, lb = lb, ub = ub, names = names, types = variable_types)
		problem.linear_constraints.add(lin_expr = constraints, senses = constraint_sense, rhs = constraint_rhs, names = constraint_names)

		problem.set_log_stream(None)
		problem.set_results_stream(None)
		problem.set_error_stream(None)
		problem.solve()
		
		values = problem.solution.get_values()

		#reconfigures vehicle and passenger info due to reassignments
		for ind in range(len(names)):
			var = names[ind]
			first_start = var.find('(') + 1 #13
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if passenger in R_prime:
				if values[ind] == 1:
					if passenger in R_A:  
						if passenger.vehicle != v:
							passenger.reassigned = 1
							if V[passenger.vehicle] in V_P:
								#reassigned passenger was reassigned from being picked up, and is being picked up by a different vehicle
								V_I.add(V[passenger.vehicle])
								V_P.remove(V[passenger.vehicle])
								V[passenger.vehicle].picking_up = None
								V[passenger.vehicle].state = 'idle'
								V[passenger.vehicle].passengers = []
								V[passenger.vehicle].serving = None
							else: #reassigned passenger must've come from a vehicle in V_D
								V[passenger.vehicle].next = None

		#takes care of all passenger assignments
		for ind in range(len(names)): 
			var = names[ind]
			first_start = var.find('(') + 1
			first_end = var.find(',') - 1
			second_start = var.find(',') + 1
			second_end = var.find(')') - 1 #len(var) - 1
			p = int(var[first_start: first_end + 1])
			v = int(var[second_start: second_end + 1])
			passenger = R[p]
			vehicle = V[v]
			if values[ind] == 1: 
				if passenger in R_prime and not(p in vehicle.passengers or p == vehicle.next):
					R_A.add(passenger)
					passenger.state = 'assigned'
					passenger.vehicle = v

					if passenger in R_U:
						R_U.remove(passenger)

					if vehicle in V_I: #passenger will be picked up immediately next
						V_I.remove(vehicle)
						V_P.add(vehicle)
						V_P_s.add(vehicle)
						vehicle.passengers = [p]
						vehicle.picking_up = 0
						vehicle.serving = 0
						vehicle.state = 'picking up'

					elif vehicle in V_P:
						vehicle.passengers = [p]
						vehicle.picking_up = 0
						vehicle.serving = 0			
													
					else: #passenger will be picked up after the next passenger is dropped off
						vehicle.next = p
						V_has_next.add(vehicle)

		for passenger in R_U:
			passenger.wait += time_interval


#<---------------------------------------------------------------------------------------------------------------------------------------------------



	empty_km_1 = 0.0 #includes all empty vehicles and rideshares going to pick up passenger 2
	empty_km_2 = 0.0 #only includes empty vehicles
	total_km = 0.0

	#set constants
	phi = drop_off_pen
	delta = reassign_pen
	gamma = wait_pen
	psi1 = pass1_pen
	psi2 = pass2_pen
	T = time_length
	t = time_interval
	t_int = time_interval
	distance_travel = t_int * vehicle_speed

	#initiate passenger sets
	R, R_U, R_A, R_IV, R_IV_1, R_IV_2, R_S, R_prime = [], set(), set(), set(), set(), set(), set(), set()
	for p in range(num_passengers):
		passenger = Passenger(p, x_max, y_max, T)
		R.append(passenger)
		if passenger.appear <= time_interval:
			R_prime.add(passenger)
			R_U.add(passenger)

	#initiate vehicle sets
	V, V_I, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_has_next = [], set(), set(), set(), set(), set(), set(), set(), set()
	V_prime = V #this is specific to this model; may change so I'm distinguishing V_prime and V
	for v in range(num_vehicles):
		vehicle = Vehicle(v, x_max, y_max)
		V.append(vehicle)
		V_I.add(vehicle)


	##for testing if rideshares work
	# R[0].x, R[0].y = 9, 3
	# R[0].o = (9, 3)
	# R[0].d = (1.08, 4.71)
	# R[0].appear = 22

	# R[1].x, R[1].y = 9, 2
	# R[1].o = (9, 2)
	# R[1].d = (1, 1)
	# R[1].appear = 22

	# V[0].x, V[0].y = 9.78454795137,2.44751492573

	#loop

	while t <= T:
		# print('Modeling time = ' + str(t))
		# for passenger in R_prime:
		# 	print('Passenger #' + str(passenger.num) + ' pos: (' + str(passenger.x) + ',' + str(passenger.y) + ')   dest: ', passenger.d, '   dist: ', point_dist((passenger.x, passenger.y), passenger.d), '  appearing ', passenger.appear)
		# for vehicle in V:
		# 	print('Vehicle #' + str(vehicle.num) + ' pos: (' + str(vehicle.x) + ',' + str(vehicle.y) + ')')
		# for vehicle in V:
		# 	if vehicle not in V_I:
		# 		print('test', vehicle.num, vehicle.passengers, vehicle.next, vehicle.serving, vehicle.picking_up, R[vehicle.passengers[0]].vehicle)
		
		if rideshare:
		
			total_km += update_in_vehicle(R, R_IV, R_IV_1, R_IV_2, R_S, V_I, V_D, V_D_s, V_D_r, V_has_next)
			e1, e2 = update_assigned(R, R_A, R_IV, R_IV_1, R_IV_2, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, t)
			empty_km_1 += e1
			empty_km_2 += e2
			total_km += e1
			# print('R_U' , R_U)
			# print('R_A' , R_A)
			# print('R_prime', R_prime)

			# print()

			# print('R_IV', R_IV)
			# print('R_S' , R_S)

			# print()

			# print('V_I' , V_I)
			# print('V_P' , V_P)
			# print('V_P_s' , V_P_s)
			# print('V_P_r' , V_P_r)
			# print('V_D' , V_D)
			# print('V_D_s' , V_D_s)
			# print('V_D_r' , V_D_r)
			# print('V_prime', V_prime)
			# print('V_has_next', V_has_next)
		else:
			total_km += update_in_vehicle_s(R, R_IV, R_S, V_I, V_D)
			e1, e2 = update_assigned_s(R, R_A, R_IV, V_P, V_D, t)
			empty_km_1 += e1
			empty_km_2 += e2
			total_km += e1

			# print('R_U' , R_U)
			# print('R_A' , R_A)
			# print('R_prime', R_prime)

			# print()

			# print('R_IV', R_IV)
			# print('R_S' , R_S)

			# print()

			# print('V_I' , V_I)
			# print('V_P' , V_P)
			# print('V_D' , V_D)
			# print('V_prime', V_prime)

		# for vehicle in V:
		# 	for p in vehicle.passengers:
		# 		print(vehicle, vehicle.num, ' is ' + vehicle.state,  R[p], R[p].num)
		# 	if vehicle.next is not None:
		# 		print(vehicle, vehicle.num, ' will eventually pick up ', R[vehicle.next], vehicle.next)
		# 	print(vehicle.num, vehicle.passengers, vehicle.next, vehicle.serving, vehicle.picking_up)


		if rideshare:
			update_unassigned(R, R_A, R_IV, R_IV_1, R_IV_2, R_prime, V, V_P, V_P_s, V_P_r, V_D, V_D_s, V_D_r, V_prime, V_has_next)
		else:
			update_unassigned_s(R, R_A, R_IV, R_prime, V, V_P, V_prime)

		t += t_int
		for passenger in R:
			if passenger.appear <= t and passenger.appear > t - t_int:
				R_prime.add(passenger)
				R_U.add(passenger)

		if len(R_S) == len(R):
			break

	wait_times = []
	overall_average = 0
	for passenger in R:
		wait_times.append(passenger.wait)
		overall_average += passenger.picked_up

	overall_average /= len(R)

	print('finished')
	print('time to finish: ' + str(t) + ', passengers served: ' + str(len(R_S)) + ', empty km 1 driven: ' + str(empty_km_1) + ', empty km 2 driven: ' + str(empty_km_2) + ', total km driven ' + str(total_km))
	return t, len(R_S), empty_km_1, empty_km_2, total_km, overall_average, wait_times

class Passenger:
	def __init__(self, num, x_max, y_max, time_horizon):
		self.num = num
		self.x = random.random() * x_max
		self.y = random.random() * y_max
		self.o = (self.x, self.y)
		self.d = (round(random.random() * x_max, 2), round(random.random() * y_max, 2))
		self.state = 'unassigned'
		self.vehicle = None #vehicle number that is serving it; not the vehicle object!
		self.reassigned = 0
		self.wait = 0
		self.appear = random.random() * time_horizon * 0.7
		# self.appear = 0 #for testing
		self.picked_up = 0
		self.extra = 0

class Vehicle:
	def __init__(self, num, x_max, y_max):
		self.num = num
		self.x = random.random() * x_max
		self.y = random.random() * y_max
		self.state = 'idle'
		self.in_vehicle = [] #for rideshare cases only
		self.passengers = [] #passenger number being served; not the passenger objects!
		self.picking_up = None #index in self.passengers that is being picked up
		self.serving = None #index in self.passengers that is getting dropped off next
		self.next = None #passenger number that will be picked up after current passengers are all dropped off

def distance(passenger, vehicle): #really doesn't matter which order arguments are given
	x_d = passenger.x - vehicle.x 
	y_d = passenger.y - vehicle.y
	return math.sqrt(x_d**2 + y_d**2)

def dist_to_d(passenger, vehicle): #order of arguments does matter
	x_d = passenger.d[0] - vehicle.x
	y_d = passenger.d[1] - vehicle.y
	return math.sqrt(x_d**2 + y_d**2)

def point_dist(p1, p2):
	x_d = p1[0] - p2[0]
	y_d = p1[1] - p2[1]
	return math.sqrt(x_d**2 + y_d**2)

#Choose inputs here
number_of_passengers = 400
number_of_vehicles = 30
vehicle_speed = 60. #kmh 55 or 60 default
x_size = 10. #km
y_size = 10. #km
run_horizon = 2. #hours
update_interval = 10. #seconds
dropoff_reasssignment_penalty = 1
reassignment_penalty = 1. #km * seconds
waiting_penalty = .05 #km/seconds
pass1_distance_pen = 1.5
pass2_distance_pen = 1.1
rideshare_flat_penalty = 1.8
rideshare = True 
zeta = 0.5

#simulation is calculated in km and seconds
vehicle_speed /= 3600. #kms
run_horizon *= 3600. #s

# time, served, empty1, empty2, total, average_waits, waits = simulate_rideshare(number_of_passengers, number_of_vehicles, vehicle_speed, x_size, y_size, run_horizon, update_interval, dropoff_reasssignment_penalty, reassignment_penalty, waiting_penalty, pass1_distance_pen, pass2_distance_pen, rideshare_flat_penalty, rideshare, zeta)

# passengers_used = []
# vehicles_used = []
# vehicle_speed_used = []
# sim_x_size_used = []
# sim_y_size_used = []
# run_horizon_used = []
# update_interval_used = []
# dropoff_reasssignment_penalty_used = []
# reassignment_penalty_used = []
# waiting_penalty_used = []
# pass1_distance_pen_used = []
# pass2_distance_pen_used = []
# rideshare_flat_penalty_used = []
# rideshare_allowed = []
# zetas = []

# variables = [passengers_used, vehicles_used, vehicle_speed_used, sim_x_size_used, sim_y_size_used, run_horizon_used, 
# 			update_interval_used, dropoff_reasssignment_penalty_used, reassignment_penalty_used, waiting_penalty_used,
# 			pass1_distance_pen_used, pass2_distance_pen_used, rideshare_flat_penalty_used, rideshare_allowed, zetas]

# run_time = []
# num_served = []
# empty_km_1 = []
# empty_km_2 = []
# total_km = []
# averages = []
# wait_times = []

# results = [run_time, num_served, empty_km_1, empty_km_2, total_km, averages, wait_times]

# passengers_used.append(number_of_passengers)
# vehicles_used.append(number_of_vehicles)
# vehicle_speed_used.append(vehicle_speed)
# sim_x_size_used.append(x_size)
# sim_y_size_used.append(y_size)
# run_horizon_used.append(run_horizon)
# update_interval_used.append(update_interval)
# dropoff_reasssignment_penalty_used.append(dropoff_reasssignment_penalty)
# reassignment_penalty_used.append(reassignment_penalty)
# waiting_penalty_used.append(waiting_penalty)
# pass1_distance_pen_used.append(pass1_distance_pen)
# pass2_distance_pen_used.append(pass2_distance_pen)
# rideshare_flat_penalty_used.append(rideshare_flat_penalty)
# zetas.append(zeta)
# rideshare_allowed.append(rideshare)

# run_time.append(time)
# num_served.append(served)
# empty_km_1.append(empty1)
# empty_km_2.append(empty2)
# total_km.append(total)
# averages.append(average_waits)
# wait_times.append(waits)

# with open ('csvfile.csv','a') as file:
# 	writer = csv.writer(file)
# 	temp = ['passengers', 'vehicles', 'vehicle speed', 'x-dim', 'y-dim', 'time horizon', 'update interval', 'immediate pickup penalty', 
# 	'reassignment penalty', 'waiting penalty', 'pass1 penalty', 'pass2 penalty', 'rideshare flat penalty', 'zeta', 'rideshares', 'run time', 
# 	'number served', 'empty km 1', 'empty km 2', 'total km', 'average wait', 'wait times']
# 	writer.writerow(temp)

# 	for i in range(len(variables[0])):
# 		res = []
# 		for j in range(len(variables)):
# 			res.append(str(variables[j][i]))
# 		for j in range(len(results)):
# 			res.append(str(results[j][i]))
# 		writer.writerow(res)