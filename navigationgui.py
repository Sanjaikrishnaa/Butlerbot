#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from PyQt5 import QtCore, QtGui, QtWidgets
import time

class Ui_host(Node):
    def __init__(self):
        super().__init__('host_node')
        self.orders = [] 
        self.navigator = BasicNavigator()
        self.init_navigation()
        self.conf = False
        self.confirmations = {}  
        self.cancelled_orders = {} 
        self.timeout_duration = 60
        self.start_time = None  

    def init_navigation(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def setupUi(self, host):
        host.setObjectName("host")
        host.resize(471, 267)
        self.verticalLayoutWidget = QtWidgets.QWidget(host)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(19, -1, 211, 261))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton_3 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout.addWidget(self.pushButton_3)
        self.pushButton_2 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(host)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(260, 0, 191, 261))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.pushButton_5 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_5.setObjectName("pushButton_5")
        self.verticalLayout_2.addWidget(self.pushButton_5)
        self.pushButton_6 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_6.setObjectName("pushButton_6")
        self.verticalLayout_2.addWidget(self.pushButton_6)
        self.pushButton_4 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_4.setObjectName("pushButton_4")
        self.verticalLayout_2.addWidget(self.pushButton_4)
        self.pushButton.clicked.connect(self.rec)
        self.pushButton_2.clicked.connect(self.t2)
        self.pushButton_3.clicked.connect(self.kit)
        self.pushButton_4.clicked.connect(self.cel)
        self.pushButton_5.clicked.connect(self.t1)
        self.pushButton_6.clicked.connect(self.t3)
       
        self.retranslateUi(host)
        QtCore.QMetaObject.connectSlotsByName(host)

    def retranslateUi(self, host):
        _translate = QtCore.QCoreApplication.translate
        host.setWindowTitle(_translate("host", "Host"))
        self.pushButton_3.setText(_translate("host", "kitchen"))
        self.pushButton_2.setText(_translate("host", "table 2"))
        self.pushButton.setText(_translate("host", "received"))
        self.pushButton_5.setText(_translate("host", "table 1"))
        self.pushButton_6.setText(_translate("host", "table 3"))
        self.pushButton_4.setText(_translate("host", "cancel"))

    
    def loc(self, pt):
        goalpose = PoseStamped()
        goalpose.header.frame_id = 'map'
        goalpose.header.stamp = self.navigator.get_clock().now().to_msg()
        goalpose.pose.position.x = pt[0]
        goalpose.pose.position.y = pt[1]
        goalpose.pose.orientation.z = pt[2]
        goalpose.pose.orientation.w = pt[3]
        return goalpose

    def goto(self, place):
        home = [0.0, 0.0, 0.0, 1.0]
        way1 = [3.720697719981621, -0.030661988244537752, 0.575688303432184, 0.8176692346489342]
        way2 = [3.470992303474623, 1.9638691635159462, 0.9796801460466589, 0.20056622707224964]
        kitchen = [0.35526450298624074, 1.9716239362763872, -0.0767681577868054, 0.99704897068801]
        table1 = [2.7981717746916286, -1.587278072269716, -0.9999474835775871, 0.010248418748817165]
        table2 = [1.172111409938088, -4.564510694850535, 0.008710036375974707, 0.999962066913705]
        table3 = [2.960650129290974, -4.5343618969923085, 0.9980635726884279, 0.062202129163004515]

        if place == "kitchen":
            waypoints = [self.loc(way2),self.loc(kitchen)]
        elif place == "table1":
            waypoints = [self.loc(table1)]
        elif place == "table2":
            waypoints = [self.loc(table2)]
        elif place == "table3":
            waypoints = [self.loc(table3)]
        elif place == "home":
            waypoints = [self.loc(home)]
        elif place == "received":
            self.conf = True
        else:
            print(f"Unknown place: {place}")
            return

        self.navigator.goThroughPoses(waypoints)
        self.start_time = time.time()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(
                    'Estimated time to complete current route: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete!')
            self.start_time = time.time()
            
        elif result == TaskResult.CANCELED:
            print('Navigation was canceled.')

        elif result == TaskResult.FAILED:
            print('Navigation failed.')

    def rec(self):
        self.conf = True
        if len(self.orders)!= 0:
            self.start_delivery()
        else:
            self.goto("home")

    def kit(self):
        self.goto("kitchen")
        self.wait_for_confirmation()

    def t1(self):
        self.place_order("table1")

    def t2(self):
        self.place_order("table2")

    def t3(self):
        self.place_order("table3")

    def wait_for_confirmation(self):
        while time.time() - self.start_time < self.timeout_duration:
            if self.conf:
                self.goto("home")
                return
        print("Timeout! No confirmation received. Returning to home.")
        self.goto("home")

    def place_order(self, table):
        if table not in self.cancelled_orders:
            self.orders.append(table)
            print(self.orders)
            print(f"Order placed for {table}")
        else:
            print(f"Order for {table} is canceled")
    
    def start_delivery(self):
        if not self.orders:
            print("No orders to deliver.")
            return

        print("Going to Kitchen first...")
        self.goto("kitchen")
        for i in self.orders:
            self.goto(i)
        
        while self.orders:
            current_order = self.orders.pop(0)  # Get the next order
            
            # Skip canceled orders
            if current_order in self.cancelled_orders:
                print(f"Skipping canceled order for {current_order}.")
                continue

            print(f"Going to {current_order}")
            self.goto(current_order)

            start_time = time.time()
            confirmed = False

            # Wait for confirmation within timeout
            while time.time() - start_time < self.timeout_duration:
                if self.confirmations.get(current_order, False):
                    confirmed = True
                    print(f"Order for {current_order} confirmed!")
                    break

                time.sleep(1)  # Poll every second

            if not confirmed:
                print(f"No confirmation for {current_order}. Going kitchen.")
                self.goto("kitchen")
                self.goto("home")
                break

        if not self.orders:
            print("All orders processed. Returning to Home.")
            self.goto("home")

    def cel(self):
        # Open cancel dialog
        self.cancel_dialog = CancelDialog(self)
        self.cancel_dialog.exec_()

class CancelDialog(QtWidgets.QDialog):
    def __init__(self, ui_host):
        super().__init__()
        self.ui_host = ui_host
        self.setWindowTitle("Cancel Order")
        self.setGeometry(100, 100, 200, 150)

        # Create cancel buttons for specific tables
        self.layout = QtWidgets.QVBoxLayout()
        self.table1_cancel_button = QtWidgets.QPushButton("Cancel Table 1 Order", self)
        self.table2_cancel_button = QtWidgets.QPushButton("Cancel Table 2 Order", self)
        self.table3_cancel_button = QtWidgets.QPushButton("Cancel Table 3 Order", self)
        self.kitchen_cancel_button = QtWidgets.QPushButton("Cancel Kitchen Order", self)
        
        # Add buttons to layout
        self.layout.addWidget(self.table1_cancel_button)
        self.layout.addWidget(self.table2_cancel_button)
        self.layout.addWidget(self.table3_cancel_button)
        self.layout.addWidget(self.kitchen_cancel_button)
        self.setLayout(self.layout)
        
        # Connect buttons to remove specific orders
        self.table1_cancel_button.clicked.connect(lambda: self.cancel_order("table_1"))
        self.table2_cancel_button.clicked.connect(lambda: self.cancel_order("table_2"))
        self.table3_cancel_button.clicked.connect(lambda: self.cancel_order("table_3"))
        self.kitchen_cancel_button.clicked.connect(lambda: self.cancel_order("kitchen"))

    def cancel_order(self, place):
        if place in self.ui_host.orders:
            self.ui_host.cancelled_orders[place] = True  # Mark the order as canceled
            print(f"{place} order canceled.")
        else:
            print(f"{place} order does not exist.")
        self.close() 
            


if __name__ == "__main__":
    rclpy.init()
    import sys
    app = QtWidgets.QApplication(sys.argv)
    host = QtWidgets.QWidget()
    ui = Ui_host()
    ui.setupUi(host)
    host.show()
    sys.exit(app.exec_())
