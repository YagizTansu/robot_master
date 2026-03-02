#!/usr/bin/env python3
"""
Graph Visualization Node
RViz'de navigation graph'ını görselleştirir
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import sys


class GraphVisualizerNode(Node):
    def __init__(self, graph_file):
        super().__init__('graph_visualizer')
        
        self.publisher = self.create_publisher(
            MarkerArray, 
            '/navigation_graph_markers', 
            10
        )
        
        # Graph'ı yükle
        self.graph_data = self.load_graph(graph_file)
        
        # Timer - her saniye publish et
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f'Graph visualizer started with {len(self.graph_data["nodes"])} nodes')
        
    def load_graph(self, file_path):
        """JSON dosyasından graph'ı yükle"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            return data['graph']
        except Exception as e:
            self.get_logger().error(f'Failed to load graph: {e}')
            return {'nodes': [], 'edges': []}
    
    def publish_markers(self):
        """Graph marker'larını publish et"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Node'ları görselleştir (Küreler)
        for node in self.graph_data['nodes']:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'graph_nodes'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Pozisyon
            marker.pose.position.x = node['x']
            marker.pose.position.y = node['y']
            marker.pose.position.z = node['z'] + 0.2  # Biraz yükselt
            marker.pose.orientation.w = 1.0
            
            # Boyut
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            # Renk (Tip göre)
            if node['type'] == 'waypoint':
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
            elif node['type'] == 'storage':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif node['type'] == 'charging':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker.lifetime.sec = 0  # Sonsuz
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Node text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'graph_labels'
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = node['x']
            text_marker.pose.position.y = node['y']
            text_marker.pose.position.z = node['z'] + 0.6  # Text üstte
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = node['id']
            text_marker.scale.z = 0.3  # Text boyutu
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
            marker_id += 1
        
        # Edge'leri görselleştir (Çizgiler)
        node_dict = {n['id']: n for n in self.graph_data['nodes']}
        
        for edge in self.graph_data['edges']:
            if edge['from'] not in node_dict or edge['to'] not in node_dict:
                continue
                
            from_node = node_dict[edge['from']]
            to_node = node_dict[edge['to']]
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'graph_edges'
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Başlangıç ve bitiş noktaları
            p1 = Point()
            p1.x = from_node['x']
            p1.y = from_node['y']
            p1.z = from_node['z'] + 0.1
            
            p2 = Point()
            p2.x = to_node['x']
            p2.y = to_node['y']
            p2.z = to_node['z'] + 0.1
            
            marker.points = [p1, p2]
            
            # Çizgi kalınlığı
            marker.scale.x = 0.05
            
            # Renk (Bidirectional ise yeşil, değilse kırmızı)
            if edge.get('bidirectional', False):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 0.6
            marker.lifetime.sec = 0
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        # Publish
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    # Graph dosya yolunu al
    if len(sys.argv) < 2:
        print("Usage: ros2 run robot_navigation visualize_graph_rviz.py <graph_json_file>")
        sys.exit(1)
    
    graph_file = sys.argv[1]
    
    node = GraphVisualizerNode(graph_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
