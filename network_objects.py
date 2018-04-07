import numpy as np
import operator
from collections import defaultdict

class Node(object):
    def __init__(self, index, id, coordinates):
        self.index = index
        self.id = id
        self.coordinates = coordinates
        self.neighbours = []
        self.shortest_paths = {}

    def get_description(self):
        return [self.index, self.id, self.coordinates, self.neighbours]


class Link(object):
    def __init__(self, id, source, target, capacity):
        self.id = id
        self.source = source
        self.target = target
        self.index_pair = []
        self.setup_cost = None
        self.capacity = capacity
        self.cost = None
        self.used_capacity = 0

    def get_description(self):
        return [self.id, self.source, self.target, self.setup_cost, self.capacity]


class Demand(object):
    def __init__(self, id, source, target, demand_value):
        self.id = id
        self.source = source
        self.target = target
        self.demand_value = demand_value

    def get_description(self):
        return [self.id, self.source, self.target, self.demand_value]


class Network(object):
    def __init__(self):
        self.nodes = []
        self.links = []
        self.demands = {}
        self.not_distributed = {}
        self.most_used_nodes = defaultdict(int)
        self.most_used_node = None
        self.link_distance = {}
        self.link_cost = {}
        self.final_paths = defaultdict(list)
        self.is_link_cost_used = 0
        self.new_link_id = 777777
        self.links_deployed = 0
        self.total_money_spent = 0

    def get_neighbours(self):
        for node in self.nodes:
            for link in self.links:
                if node.index in link.index_pair:
                    if link.index_pair.index(node.index) == 1:
                        if self.get_node_by_index(link.index_pair[0]) not in node.neighbours:
                            node.neighbours.append(self.get_node_by_index(link.index_pair[0]))
                    else:
                        if self.get_node_by_index(link.index_pair[1]) not in node.neighbours:
                            node.neighbours.append(self.get_node_by_index(link.index_pair[1]))

    def count_distance(self, node1, node2):
        distance = np.arccos((np.sin(node1.coordinates['x']) * np.sin(node2.coordinates['x'])) + (
            np.cos(node1.coordinates['x']) * np.cos(node2.coordinates['x']) * np.cos(
                np.abs(node1.coordinates['y'] - node2.coordinates['y']))))
        distance *= 111.195

        return np.around(distance, 4)

    def count_cost(self, distance):
        cost = 2 * pow(distance, 4)
        return np.around(cost, 2)

    def count_flow_values_and_cost(self):
        for node1 in self.nodes:
            for node2 in self.nodes:
                if node1 != node2:
                    self.demands[self.nodes.index(node1), self.nodes.index(node2)] = 10 * abs(
                        self.nodes.index(node1) - self.nodes.index(node2))
                    distance = self.count_distance(node1, node2)
                    self.link_distance[self.nodes.index(node1), self.nodes.index(node2)] = distance
                    self.link_cost[self.nodes.index(node1), self.nodes.index(node2)] = 1000 * (
                        self.count_cost(distance / 100))

    def fill_link_index_pair(self):
        for link in self.links:
            link.index_pair = [self.get_object_by_id(link.source).index, self.get_object_by_id(link.target).index]

    def is_connected(self, index1, index2):
        for link in self.links:
            if link.index_pair == [index1, index2] or link.index_pair == [index2, index1]: return True
        return False

    def get_nodes(self):
        return [node.get_description() for node in self.nodes]

    def get_links(self):
        return [link.get_description() for link in self.links]

    def get_demands(self):
        return [demand.get_description() for demand in self.demands]

    def get_node_by_index(self, index):
        return [node for node in self.nodes if node.index == index][0]

    def get_link_by_index_pair(self, source_index, target_index):
        for link in self.links:
            if link.index_pair == [source_index, target_index]: return link
        return None

    def get_object_by_id(self, id):
        if id.startswith('Link_'):
            return [link for link in self.links if link.id == id][0]
        else:
            return [node for node in self.nodes if node.id == id][0]

    def get_link_by_source(self, source):
        return [link for link in self.links if link.source == source]

    def get_link_by_target(self, target):
        return [link for link in self.links if link.target == target]

    def get_link_by_source_and_target(self, source, target):
        for link in self.links:
            if (link.source == source and link.target == target):
                return link
        return False

    def get_demand_by_source(self, source):
        return [demand for demand in self.demands if demand.source == source]

    def get_demand_by_target(self, target):
        return [demand for demand in self.demands if demand.target == target]

    def get_node_coordinates(self, id):
        return self.get_object_by_id(id).coordinates

    def get_source(self, id):
        return self.get_object_by_id(id).source

    def get_target(self, id):
        return self.get_object_by_id(id).target

    def get_node_by_name(self, name):
        for node in self.nodes:
            if node.id == name:
                return node

    def count_existing_link_cost(self):
        for link in self.links:
            link.cost = abs((self.get_node_by_name(link.source).index - self.get_node_by_name(link.target).index)) * 10

    # KROK 1 ALGORYTMU - ROZLOZENIE RUCHU
    def distribute_traffic(self):
        self.find_the_shortest_paths()
        self.distribute_traffic_between_neighbours()
        self.distribute_traffic_via_shortest_paths()
        print "Distributed: " + str(len(self.final_paths))
        print "Not distributed: " + str(len(self.not_distributed))
        if len(self.not_distributed):
            self.demands = self.not_distributed
            self.search_for_the_most_used_node()
            for node1, node2 in self.demands:
                self.get_node_by_index(node1).neighbours = []
                self.get_node_by_index(node2).neighbours = []
            self.add_first_link()
            self.find_the_shortest_paths()
            self.distribute_traffic_via_shortest_paths()
        while self.demands:
            self.prepare_for_links_deployment()
            for demand in self.demands:
                node = self.get_node_by_index(demand[0])
                node.shortest_paths = self.dijkstra2(self.nodes, node.id)
                print "--------------------------"
                print("Processing demand: {} to {}".format(self.get_node_by_index(demand[0]).id,
                                                           self.get_node_by_index(demand[1]).id))
                target = self.get_node_by_index(demand[1]).id

                while node.shortest_paths[self.get_node_by_index(demand[1]).id]:
                    source = node.shortest_paths[self.get_node_by_index(demand[1]).id].pop()
                    if self.get_link_by_source_and_target(source, target) == False:
                        self.links.append(Link("Link_777777", source, target, 10000))
                        self.links_deployed +=1
                        self.total_money_spent += self.link_cost[self.get_object_by_id(source).index, self.get_object_by_id(target).index]
                        self.link_cost[self.get_object_by_id(source).index, self.get_object_by_id(target).index] = 0
                        #print self.demands
                        target = source
                self.demands.pop(demand)
                self.final_paths[demand].append(self.get_object_by_id("Link_777777"))
                break

    def prepare_for_links_deployment(self):
        for node in self.nodes:
            node.shortest_paths = {}
        self.is_link_cost_used = 1
        for node in self.nodes:
            for neighbour in self.nodes:
                if node != neighbour:
                    node.neighbours.append(neighbour)

    def find_the_shortest_paths(self):
        for node in self.nodes:
            node.shortest_paths = self.dijkstra(self.nodes, node.id)

    def dijkstra(self, nodes_list, initial):
        visited = {initial: 0}
        path = defaultdict(list)
        nodes = set(nodes_list)
        while nodes:
            min_node = None

            for node in nodes:
                if node.id in visited:
                    if min_node is None:
                        min_node = node
                    elif visited[node.id] < visited[min_node.id]:
                        min_node = node

            if min_node is None:
                break

            nodes.remove(min_node)
            current_weight = visited[min_node.id]

            for neighbour in self.get_node_by_name(min_node.id).neighbours:
                if self.is_link_cost_used == 1:
                    weight = current_weight + self.link_cost[min_node.index, neighbour.index]
                else:
                    weight = current_weight + 1

                if neighbour.id not in visited or weight < visited[neighbour.id]:
                    visited[neighbour.id] = weight
                    for each in path[min_node.id]:
                        path[neighbour.id].append(each)
                    path[neighbour.id].append(min_node.id)
        return path

    def dijkstra2(self, nodes_list, initial):
        visited = {initial: 0}
        path = defaultdict(list)
        nodes = set(nodes_list)
        while nodes:
            min_node = None

            for node in nodes:
                if node.id in visited:
                    if min_node is None:
                        min_node = node
                    elif visited[node.id] < visited[min_node.id]:
                        min_node = node

            if min_node is None:
                break

            nodes.remove(min_node)
            current_weight = visited[min_node.id]

            for neighbour in self.get_node_by_name(min_node.id).neighbours:
                if self.is_link_cost_used == 1:
                    weight = current_weight + self.link_cost[min_node.index, neighbour.index]
                else:
                    weight = current_weight + 1

                if neighbour.id not in visited or weight < visited[neighbour.id]:
                    visited[neighbour.id] = weight
                    path[neighbour.id].append(min_node.id)
        return path

    def is_enough_capacity(self, links, demand):
        result = False
        if links:
            for link in links:
                if self.demands[demand] <= link.capacity:
                    pass
                else:
                    break
            else:
                result = True
        return result

    def put_traffic_into_link(self, link, demand):
        link.capacity -= self.demands[demand]
        self.final_paths[demand].append(link)

    def distribute_traffic_between_neighbours(self):
        for demand in self.demands:
            if self.is_connected(demand[0], demand[1]) and self.is_enough_capacity(
                    [self.get_link_by_index_pair(demand[0], demand[1])],
                    demand) and demand not in self.final_paths.keys():
                self.put_traffic_into_link(self.get_link_by_index_pair(demand[0], demand[1]), demand)

    def distribute_traffic_between_neighbours_2(self):
        for demand in self.not_distributed:
            if self.is_connected(demand[0], demand[1]) and self.is_enough_capacity(
                    [self.get_link_by_index_pair(demand[0], demand[1])],
                    demand) and demand not in self.final_paths.keys():
                self.put_traffic_into_link(self.get_link_by_index_pair(demand[0], demand[1]), demand)

    def distribute_traffic_via_shortest_paths(self):
        for demand in self.demands:
            if demand not in self.final_paths:
                if self.is_enough_capacity(self.parse_shortest_path_to_links_list(demand[0], demand[1]), demand):
                    for link in self.parse_shortest_path_to_links_list(demand[0], demand[1]):
                        self.put_traffic_into_link(link, demand)
                else:
                    self.not_distributed[demand] = self.demands[demand]

    def print_final_distribution(self):
        for demand, path in self.final_paths.iteritems():
            print "\nRuch miedzy {}, {} o wartosci {} przez linki: ".format(self.get_node_by_index(demand[0]).id,
                                                                            self.get_node_by_index(demand[1]).id,
                                                                            self.demands[demand])
            for link in path:
                print "({}, {}), ".format(link.source, link.target),

    def parse_shortest_path_to_links_list(self, start, end):
        links_list = []
        node = self.get_node_by_index(start)
        end_id = self.get_node_by_index(end).id
        if node.shortest_paths[end_id]:
            for current, next in zip(node.shortest_paths[end_id], node.shortest_paths[end_id][1:]):
                links_list.append(self.get_link_by_source_and_target(current, next))
            links_list.append(self.get_link_by_source_and_target(node.shortest_paths[end_id][-1], end_id))
        return links_list

    def search_for_the_most_used_node(self):
        self.most_used_node = None
        self.most_used_nodes = defaultdict(int)
        for node1, node2 in self.not_distributed:
            self.most_used_nodes[node1] += 1
            self.most_used_nodes[node2] += 1
        sorted_x = sorted(self.most_used_nodes.items(), key=operator.itemgetter(1))
        self.most_used_nodes = sorted_x
        print "MOST USED NODE: " + str(self.most_used_nodes[-1][0])
        self.most_used_node = self.most_used_nodes[-1][0]

    def add_first_link(self):
        nodes_to_evaluate_cost = []
        for node1, node2 in self.not_distributed:
            if node1 == self.most_used_node and node2 not in nodes_to_evaluate_cost:
                    nodes_to_evaluate_cost.append(node2)
            if node2 == self.most_used_node and node1 not in nodes_to_evaluate_cost:
                nodes_to_evaluate_cost.append(node1)
        destination_node = None
        for node in nodes_to_evaluate_cost:
            if destination_node == None:
                destination_node = node
            if self.link_cost[self.most_used_node, node] < self.link_cost[self.most_used_node, destination_node]:
                destination_node = node
        self.links.append(Link('Link_777777', self.get_node_by_index(self.most_used_node).id,
                               self.get_node_by_index(destination_node).id, 10000))
        self.links_deployed += 1
        self.link_cost[self.most_used_node, destination_node] = 0
        self.get_object_by_id('Link_777777').index_pair = [self.most_used_node, destination_node]
        for node in self.nodes:
            node.neighbours = []
        print self.not_distributed
        self.not_distributed.pop((destination_node, self.most_used_node))
        self.final_paths[destination_node, self.most_used_node].append(self.get_object_by_id('Link_777777'))


