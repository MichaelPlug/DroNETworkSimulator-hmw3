from operator import ne
import numpy as np
import math
from numpy.core.defchararray import array
from numpy.core.numeric import NaN
from numpy.lib.type_check import real_if_close
from src.routing_algorithms.georouting import GeoRouting
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt


#TO FIX TO OBTAIN THE NUMBER OF DRONES (COSTANT OF CONFIG.PY)
import src.utilities.config as config #try self.simulator.n_drones


#GLOBAL THINGS

#import the library for random values
import random



#dictionary needed to establish which depot is better to go
#in normal case we haven't the need to check this information



#seed for random values, just to have consistence on values
#TODO
#eliminate after a while
random.seed(2)

#epsilon must be smaller and it represents probability for epsilon-greedy
second_epsilon = 0.05
min_epsilon = 0.05
max_epsilon = 0.25

georouting_on_next_step = True

#take a random value between 0,1
epsilon = random.random()

#normalize the random value from min_epsilon to max_epsilon
epsilon = min_epsilon + (epsilon * (max_epsilon - min_epsilon))


alpha = 0.5

gamma = 0.2

class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_action)
        setattr(self, 'count', 0)

    def feedback(self, drone, id_event, delay, outcome, depot_index=None):

        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        #print(self.drone.identifier, "----------", self.taken_actions)

        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet
        #print(self.drone.identifier, "----------", drone, id_event, delay, outcome)

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)

        print(self, drone, id_event, delay, outcome)


        try:
        	appo = self.drone.q
        except:
        	setattr(self.drone, 'q', {})

        try:
         	appo = self.drone.s
        except:
        	setattr(self.drone, 's', {})

        try:
         	appo = self.drone.v_star
        except:
        	setattr(self.drone, 'v_star', {})
        try:
         	appo = self.drone.q_depot
        except:
        	setattr(self.drone, 'q_depot', {})
        try:
            self.drone.k = self.drone.k + 1
        except:
            setattr(self.drone, 'k', 1)


        #if the packet isn't still treated, then we train system for it
        if True:

            "Doubt: i don't know the utility of this"
            if id_event in self.taken_actions:
                action = self.taken_actions[id_event]
                del self.taken_actions[id_event]
            "End of doubt"

            #if the packet is arrived isn't more valid
            if (outcome == -1):

                #we obtain a small reward
                R = -1

                #R = -2

            #if the packet arrived
            else:

                #opposite of delay
                temp = 2000 - delay

                #we obtain a linear reward based on the delay -->
                #more the delay and value more close value to 1...
                #less the delay and value more close to 2
                temp = (temp - 0) / (2000 - 0)
                #take the reward
                R = 1 + temp


                #R = 2 * (1 - delay/self.simulator.event_duration)

            #add attempts for the starting drone that has initially the packet
            #TODO
            #maybe also for all the path of packets to incentive them????







            try:

                temp_, max_q =  self.drone.s[(drone.identifier, id_event)]

            except Exception as e:

                temp_, max_q = 0, 0


            drone_cell_index = self.get_grid_and_next_grid(drone)



            #in this way we update the dictionary of the reward for the specific depot
            #it's important consider this update always, for each action done, also passing of packets
            try:

                self.drone.q_depot[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], depot_index] = self.drone.q_depot[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], depot_index] + R

            except:


                self.drone.q_depot[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], depot_index] = 10 + R


            #in this way we also consider the action of return -1
            if (temp_ == -1 or temp_ == -2):


                try:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] )


                except Exception as e:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] = 10

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 2] )


            #the packet remain to the node
            elif(temp_ == 0):


                try:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0]  = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0] + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0])

                except Exception as e:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0] = 10

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0]  = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0] + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 0])




            #the packet was passed to another drone
            elif (temp_ == 1):


                try:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  )

                except Exception as e:

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  = 10

                    self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  + alpha*(R + gamma* max_q - self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]  )




            #q[drone.identifier][self.drone.identifier] = self.drone.q[drone.identifier][self.drone.identifier] + R




            """


            # Packets that we delivered and still need a feedback
            print("Drone: ", self.drone.identifier, "---------- has delivered: ", self.taken_actions)

            # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
            # Feedback from a delivered or expired packet
            print("Drone: ", self.drone.identifier, "---------- just received a feedback:",
                  "Drone:", drone, " - id-event:", id_event, " - delay:",  delay, " - outcome:", outcome)


            """

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        # Notice all the drones have different speed, and radio performance!!
        # you know the speed, not the radio performance.
        # self.drone.speed

        # Only if you need --> several features:
        self_cell_index = self.get_grid_and_next_grid(self.drone)




        ####action = None


        try:
        	appo = self.drone.q
        except:
        	setattr(self.drone, 'q', {})

        try:
         	appo = self.drone.s
        except:
        	setattr(self.drone, 's', {})

        try:
         	appo = self.drone.v_star
        except:
        	setattr(self.drone, 'v_star', {})
        try:
         	appo = self.drone.q_depot
        except:
        	setattr(self.drone, 'q_depot', {})

        try:
        	mustGoBack = self.drone.mustGoBack
        except:
        	mustGoBack = False
        	setattr(self.drone, 'mustGoBack', mustGoBack)

        try:
        	mustGoBackEps = self.drone.mustGoBackEps
        except:
        	mustGoBackEps = False
        	setattr(self.drone, 'mustGoBackEps', mustGoBack)

        for pkt, d in opt_neighbors:
        	if d.move_routing:
        		self.drone.mustGoBack = False
        		return d

       	for pkd, d in opt_neighbors:
       		try:
       			if d.mustGoBack:
       				return d
       		except:
       			continue


        if mustGoBack:
            if mustGoBackEps:
            	distanza_depot_0 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[0])
            	distanza_depot_1 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[1])
            	if distanza_depot_0 <= distanza_depot_1:
            		bd = 0
            	else:
            		bd = 1
            else:
            	bd = self.bestDepot(self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1])
            if self.isGoingAway(bd):
            	self.drone.mustGoBack = False
            	self.drone.mustGoBackEps = False
            	return -bd -1
            else:
            	return None

        imthebest = True

        bestDrone = self.drone

        for pkt, d in opt_neighbors:
       		if d.buffer_length() > 0:
       			if self.compareQ(bestDrone, d):
       			          imthebest = False
       			          bestDrone = d
        if not imthebest:
            self.mustGoBack = False
            return bestDrone

        #we take our distance from the depot
        #best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
        #best_drone_distance_from_depot = self.compute_distance_to_trajectory_s()
        #initially drone closest is us (we take to the depot the
        #packet without any help)
        best_drone = None
        #we initialize an empty list, to find the maximum value
        l = []

        #we inizialize a temporanery variable to find maximum value from a list
        m = 0

        #we generate a random value, for the epsilon-greedy strategy
        rand = random.random()

        try:
            n = self.drone.k
        except:
            setattr(self.drone, 'k', 0)
            n = 0

        epsilon = 1 - n/(n+1)


        #if we are in greedy case
        if (rand < 1 - epsilon):

            #we calculate what is the best action to perform, if it is
            #the action of None, -1 or pass to any neighbour

            try:

                a = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0]

            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0] = 10


                a = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0]

            try:

                b = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1]

            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1] = 12

                b = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1]

            try:

                c = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2]

            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2] = 10

                c = self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2]



            #if the best action is to maintain the packet and remain to
            #our trajectory
            if (a >= b and a >= c):

                #we calculate the maximum value of the three possible actions
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2]]
                m = max(l)

                #we set, in a global variable, that this drone
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, m)



                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m

                except:

                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m


                #do anything
                return None

            #if the better action is to go to the depot, then this means
            #that we want do the same of the previous if, in practise
            if (c >= a and c >= b):

                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2]]
                m = max(l)

                #we save this result, in practise
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)

                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m

                except:

                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m


                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose

                bd = self.bestDepot(self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1])
                if self.isGoingAway(bd):
                     return -bd-1





                try:
                     self.drone.mustGoBack = True
                except:
                     setattr(self.drone, 'mustGoBack', True)
                return None
            #if the best choice to do is to pass the packet to the neighbors
            if (b >= a and b >= c):

                "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1], self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2]]
                m = max(l)

                #we initialize two differente variables to do some things
                sum_v_star = 0
                max_v_star = 0


                "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
                #we meed to know sum of v* of all neighbors
                for hello_packet, drone_istance in opt_neighbors:

                    try:
                    	sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]
                    except:
                    	self.drone.v_star[drone_istance.identifier] = 0
                    	sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]


                try:

                    #set the v_star attribute
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star

                except:

                    self.drone.v_star[self.drone.identifier] = 0

                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star


                max_v_star = self.drone.v_star[self.drone.identifier]

                max_action = None

                "THIRD PHASE -- IDENTIFY THE MAX FOR a OF EVERY Q(S',a) (all neighbors)"
                "SLIDE 42"
                #loop for every neighbors
                for hello_packet, drone_istance in opt_neighbors:



                    try:
                    	ap = self.drone.v_star[drone_istance.identifier]
                    except:
                    	self.drone.v_star[drone_istance.identifier] = 0
                    #because we must identify max_a Q(S' , a)
                    if (self.drone.v_star[drone_istance.identifier] > max_v_star):

                        max_v_star = self.drone.v_star[drone_istance.identifier]
                        max_action = drone_istance

                #max of possible actions
                return_m = m

                "FOURTH PHASE -- SELECT THE BEST NEIGHBOR POSSIBLE, WITH"
                "HIGHEST VALUE LEARNED"
                for i in range(3):

                    for hello_packet, drone_istance in opt_neighbors:




                        drone_cell_index = self.get_grid_and_next_grid(drone_istance)

                        try:


                            if (drone_istance.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], i] > return_m):
                            	return_m = drone_istance.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], i]


                        except:


                            drone_istance.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], i] = 10


                            if (drone_istance.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], i] > return_m):

                                return_m = drone_istance.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], i]


                #save everything for the capturing of the reward in
                #successive phase of feedback
                if (max_action == None):

                     self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)

                else:

                     self.drone.mustGoBack = False
                     self.drone.s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)


                return max_action






        #in the random case (epsilon case)
        else:


            distanza_depot_1 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[0])
            #distanza_depot_1 = util.euclidean_distance(self.simulator.depot.list_of_coords[0], self.drone.next_target())


            #distanza_depot_2 = distanza_depot_2 =util.euclidean_distance(self.simulator.depot.list_of_coords[1], self.drone.next_target())
            distanza_depot_2 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[1])
            "DEPOT 1 ?? RETURN -1, DEPOT 2 ?? RETURN -2"

            if (distanza_depot_1 <= distanza_depot_2):

                best_drone_distance_from_depot = distanza_depot_1

            else:

                best_drone_distance_from_depot = distanza_depot_2



            """ arg min score  -> geographical approach, take the drone closest to the depot """

            max_action = None


            l = []


            #we take the maximum value, for the reward calculation
            try:

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0])


            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0] = 10

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 0])


            try:

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1])


            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1] = 10

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 1])


            try:

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2])


            except:

                self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2] = 10

                l.append(self.drone.q[self_cell_index[0][0], self_cell_index[0][1], self_cell_index[1][0], self_cell_index[1][1], 2])




            # Action MOVE is identified by -1
            if len(opt_neighbors) == 0:


                m = max(l)

                #we save this result, in practise
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)

                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m

                except Exception as e:

                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m


                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose
                distanza_depot_1 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[0])

                distanza_depot_2 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[1])


                if (distanza_depot_1 <= distanza_depot_2):

                        bd = 0

                else:
                	bd = 1

                if self.isGoingAway(bd):

                        return -bd -1

                try:
                     self.drone.mustGoBack = True
                except:
                     setattr(self.drone, 'mustGoBack', True)
                try:
                     self.drone.mustGoBackEps = True
                except:
                     setattr(self.drone, 'mustGoBackEps', True)
                return None


            "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
            #we take the maximum value, for the reward calculation
            #this is the ELSE part

            m = max(l)

            #we initialize two differente variables to do some things
            sum_v_star = 0
            max_v_star = 0


            "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
            #we meed to know sum of v* of all neighbors
            for hello_packet, drone_istance in opt_neighbors:


                try:


                    sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]

                except Exception as e:

                    self.drone.v_star[drone_istance.identifier] = 0

                    sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]



            try:

                #set the v_star attribute
                self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star

            except Exception as e:

                self.drone.v_star[self.drone.identifier] = 0

                self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star


            max_v_star = self.drone.v_star[self.drone.identifier]

            max_action = None

            #max of possible actions
            return_m = m






            for hpk, drone_istance in opt_neighbors:

                exp_position = hpk.cur_pos  # without estimation, a simple geographic approach



                distanza_depot_1 = util.euclidean_distance(self.simulator.depot.list_of_coords[0], self.drone.next_target())

                distanza_depot_2 =util.euclidean_distance(self.simulator.depot.list_of_coords[1], self.drone.next_target())
                "DEPOT 1 ?? RETURN -1, DEPOT 2 ?? RETURN -2"

                if (distanza_depot_1 <= distanza_depot_2):

                    exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.list_of_coords[0])

                else:

                    exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.list_of_coords[1])



                if exp_distance < best_drone_distance_from_depot:
                    best_drone_distance_from_depot = exp_distance
                    max_action = drone_istance


                    drone_cell_index = self.get_grid_and_next_grid(drone_istance)




                    try:

                        return_m = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]


                    except:

                        self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1] = 10

                        return_m = self.drone.q[drone_cell_index[0][0], drone_cell_index[0][1], drone_cell_index[1][0], drone_cell_index[1][1], 1]




            #save everything for the capturing of the reward in
            #successive phase of feedback
            if (max_action == None):


                m = max(l)

                #we set, in a global variable, that this drone
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)



                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m

                except:

                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m





            else:

                 self.drone.mustGoBack = False
                 self.drone.s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)



            return max_action


        """

        FINE

        """







    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        print(self.count)
        pass

    def get_grid_and_next_grid(self):
    	return self.get_grid_and_next_grid(self.drone)






    def get_grid_and_next_grid(self, drone):
    	actual_grid = np.asarray(drone.coords) // self.simulator.prob_size_cell
    	agx = actual_grid[0]
    	agy = actual_grid[1]
    	future_grid = np.asarray(drone.next_target()) // self.simulator.prob_size_cell
    	fgx = future_grid[0]
    	fgy = future_grid[1]
    	if agx == fgx and agy == fgy:
    		return (actual_grid, actual_grid)
    	'''
    	if (np.asarray(drone.next_target()) // self.simulator.prob_size_cell == actual_grid).any:
    		print(np.asarray(drone.next_target()) // self.simulator.prob_size_cell)
    		print(actual_grid)
    		self.count = self.count -1
    	'''


    	drone_distance_next_target = np.asarray(drone.next_target()) - np.asarray(drone.coords)
    	if drone_distance_next_target[0] > 0:
    		up = True
    		if drone_distance_next_target[1] > 0:
    			cross_point = (actual_grid + 1)*self.simulator.prob_size_cell
    			right = True
    		else:
    			cross_point = (actual_grid)*self.simulator.prob_size_cell
    			cross_point[0] = cross_point[0]*self.simulator.prob_size_cell
    			right = False
    	else:
    		up = False
    		if drone_distance_next_target[1] > 0:
    			cross_point = (actual_grid)*self.simulator.prob_size_cell
    			cross_point[1] = cross_point[1]*self.simulator.prob_size_cell
    			right = True
    		else:
    			cross_point = (actual_grid)*self.simulator.prob_size_cell
    			right = False
    	drone_distance_cross_point = cross_point - np.asarray(drone.coords)
    	if drone_distance_next_target[1] == 0:
    		grad = -1
    	else:
    		if drone_distance_cross_point[1] == 0:
    			grad = 1
    		else:
    			grad = abs(drone_distance_next_target[0]/drone_distance_next_target[1]) - abs(drone_distance_cross_point[0]/drone_distance_cross_point[1])

    	if grad > 0:
    		if right:
    			return (actual_grid, [actual_grid[0]+1, actual_grid[1]])
    		else:
    			return (actual_grid, [actual_grid[0] -1, actual_grid[1]])
    	else:
    		if up:
    			return (actual_grid, [actual_grid[0], actual_grid[1] +1])
    		else:
    			return (actual_grid, [actual_grid[0], actual_grid[1] -1])

    def isGoingAway(self, depot):
        a = util.euclidean_distance(self.simulator.depot.list_of_coords[depot], self.drone.next_target())
        b = util.euclidean_distance(self.drone.coords, self.drone.next_target())
        c = util.euclidean_distance(self.simulator.depot.list_of_coords[depot], self.drone.coords)

        if b == 0:
            return False
        if a == 0:
            return False
        if c == 0:
            return False

        import math
        arg = (b**2 + c**2 - a**2) / (2.0*b*c)
        if arg > 1.0:
            arg = 1

        if arg < -1.0:
    	    	arg = -1
        alpha = math.acos(arg)
        return alpha >= math.pi/2

    def imlating(self, cur_step):
        """ return true if exist a packet that is expiring and must be returned to the depot as soon as possible
            -> start to move manually to the depot.

            This method is optional, there is flag src.utilities.config.ROUTING_IF_EXPIRING
        """
        distanza_depot_1 = util.euclidean_distance(self.simulator.depot.list_of_coords[0], self.drone.next_target())

        distanza_depot_2 =util.euclidean_distance(self.simulator.depot.list_of_coords[1], self.drone.next_target())
        "DEPOT 1 ?? RETURN -1, DEPOT 2 ?? RETURN -2"

        if (distanza_depot_1 <= distanza_depot_2):

            dist = self.simulator.depot.list_of_coords[0]

        else:

            dist = self.simulator.depot.list_of_coords[1]


        time_to_depot = util.euclidean_distance(dist, self.drone.coords) / self.drone.speed
        event_time_to_dead = (self.drone.tightest_event_deadline - cur_step) * self.drone.simulator.time_step_duration
        return event_time_to_dead - 5 < time_to_depot <= event_time_to_dead  # 5 seconds of tolerance

    def bestDepot(self, x1, y1, x2, y2):
        try:
            q_dep1 = self.drone.q_depot[x1, y1, x2, y2, -1]
        except:
            self.drone.q_depot[x1, y1, x2, y2, -1] = 10
            q_dep1 = 10

        try:
            q_dep2 = self.drone.q_depot[x1, y1, x2, y2, -2]
        except:
            self.drone.q_depot[x1, y1, x2, y2, -2] = 10
            q_dep2 = 10

        diff = q_dep1 - q_dep2

        if diff > 0:
            return 0
        elif diff < 0:
            return 1

        #distanza_depot_0 = util.euclidean_distance(self.simulator.depot.list_of_coords[0], self.drone.next_target())
        distanza_depot_0 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[0])
        #distanza_depot_1 =util.euclidean_distance(self.simulator.depot.list_of_coords[1], self.drone.next_target())
        distanza_depot_1 = self.compute_distance_to_trajectory(self.drone, self.simulator.depot.list_of_coords[1])

        if distanza_depot_0 <= distanza_depot_1:
            return 0
        else:
            return 1

    def compareQ(self, droneA, droneB):

        gridA = self.get_grid_and_next_grid(droneA)
        gridB = self.get_grid_and_next_grid(droneB)

        x1a = gridA[0][0]
        y1a = gridA[0][1]
        x2a = gridA[1][0]
        y2a = gridA[1][1]

        x1b = gridB[0][0]
        y1b = gridB[0][1]
        x2b = gridB[1][0]
        y2b = gridB[1][1]

        try:
            q_A = droneA.q[x1a, y1a, x2a, y2a, 0]
        except:
            #droneA.q[x1a, y1a, x2a, y2a, 0] = 10
            q_A = 10

        try:
            q_B = droneB.q[x1b, y1b, x2b, y2b, 0]
        except:
            #droneB.q[x1b, y1b, x2b, y2b, 0] = 10
            q_B = 10

        return q_B > q_A

    def compute_distance_to_trajectory(self, hello_packet, depot_coords):

        #exp_position = self.compute_extimed_position(hello_packet)
        exp_position = hello_packet.coords
       # exp_position = hello_packet.cur_pos

        #MAYBE IT SHOULD BE p1 = np.array([exp_position[0][0], exp_position[0][1]])
        p1 = np.array([exp_position[0], exp_position[1]])
        nt = hello_packet.next_target()
        print(nt)
        p2 = np.array([nt[0], nt[1]])
        p3 = np.array([depot_coords[0],depot_coords[1]])

        if np.linalg.norm(p2-p1) != 0:
        	return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
        else:
        	return 0

    def compute_extimed_position(self, hello_packet):
        """ estimate the current position of the drone """

        # get known info about the neighbor drone
        hello_message_time = hello_packet.time_step_creation
        known_position = hello_packet.cur_pos
        known_speed = hello_packet.speed
        known_next_target = hello_packet.next_target

        # compute the time elapsed since the message sent and now
        # elapsed_time in seconds = elapsed_time in steps * step_duration_in_seconds
        elapsed_time = (self.simulator.cur_step - hello_message_time) * self.simulator.time_step_duration  # seconds

        # distance traveled by drone
        distance_traveled = elapsed_time * known_speed

        # direction vector
        a, b = np.asarray(known_position), np.asarray(known_next_target)
        if np.linalg.norm(b - a) != 0:
        	v_ = (b - a) / np.linalg.norm(b - a)
        else:
        	v_ = 0

        # compute the expect position
        c = a + (distance_traveled * v_)

        return tuple(c)
