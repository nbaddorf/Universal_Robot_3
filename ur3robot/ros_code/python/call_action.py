import rospy
import actionlib
import rail_mesh_icp.msg
#from rail_mesh_icp.msg import MatchTemplateAction
#from rail_mesh_icp.msg import MatchTemplateActionFeedback
#from rail_mesh_icp.msg import MatchTemplateActionGoal
#from rail_mesh_icp.msg import MatchTemplateActionResult
#from rail_mesh_icp.msg import MatchTemplateFeedback
#from rail_mesh_icp.msg import MatchTemplateGoal
#from rail_mesh_icp.msg import MatchTemplateResult

def call_actionlib():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('template_matcher_demo_node/match_template', rail_mesh_icp.msg.MatchTemplateAction)
    print("waiting for client")
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("found client")

    # Creates a goal to send to the action server.
    goal = rail_mesh_icp.msg.MatchTemplateGoal()
    print ("created goal")
    # Sends the goal to the action server.
    client.send_goal(goal)
    print ("sent goal")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("waiting for goal")
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = call_actionlib()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")