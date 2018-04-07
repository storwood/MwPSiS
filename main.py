from network_objects import Network
from SNDlib_parser import SNDlibParser
from collections import defaultdict
import time

start_time = time.time()

if __name__ == '__main__':
    network = Network()
    SNDlibParser("sndlib_network.xml").parse_to_object(network)
    network.fill_link_index_pair()
    network.get_neighbours()
    network.count_flow_values_and_cost()

    print "Amount of demands: " + str(len(network.demands))

    network.distribute_traffic()

    print "Distributed: " + str(len(network.final_paths))
    print "Not distributed: " + str(len(network.not_distributed))
    print "Total number of new links deployed: ", network.links_deployed
    print "Total money spent: ", network.total_money_spent, "PLN"
    print("Program execution: {}s".format(time.time()-start_time))