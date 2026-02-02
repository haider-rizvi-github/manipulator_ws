#!/usr/bin/python3

from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler

# Initialize ROS interaction with action server
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from manipulator_msgs.action import ManipulatorTask
import threading

# runnning ROS2 Node in separate thread
threading.Thread(target=lambda: rclpy.init()).start()

action_client = ActionClient(
    Node("alexa_interface_node"), ManipulatorTask, "task_server"
)  # name of node: alexa_interface_node,msg type action client will use:ManipulatorTask.action messages name of action server: task_server


app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, How can i help you?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)
        ).set_should_end_session(False)

        # Prepare the goal message to be sent to action server
        goal = ManipulatorTask.Goal()
        goal.task_number = 0  # sending task number 0 for launch request
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response


# Creating the classes of custom intents
class PickntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Picking up the item"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick Intent", speech_text)
        ).set_should_end_session(True)

        # Prepare the goal message to be sent to action server
        goal = ManipulatorTask.Goal()
        goal.task_number = 1  # sending task number 1 for pick intent
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response


class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Robot is going to sleep"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep Intent", speech_text)
        ).set_should_end_session(True)

        # Prepare the goal message to be sent to action server
        goal = ManipulatorTask.Goal()
        goal.task_number = 2  # sending task number 2 for sleep intent
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Robot is waking up"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake Intent", speech_text)
        ).set_should_end_session(True)

        # Prepare the goal message to be sent to action server
        goal = ManipulatorTask.Goal()
        goal.task_number = 0  # sending task number 0 for wake request
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response


# Exception Handler
class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I missed what you said. Please Repeat!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


# Create Skill Builder and register handlers
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=skill_builder.create(),
    skill_id="amzn1.ask.skill.8303b30e-fb9b-4d1f-9ec4-8cb57f872915",
    app=app,
)

skill_adapter.register(app=app, route="/alexa")


@app.route("/", methods=["GET"])
def health_check():
    return "Alexa Skill is running.", 200  # 200 is http status code for OK


if __name__ == "__main__":
    app.run()
