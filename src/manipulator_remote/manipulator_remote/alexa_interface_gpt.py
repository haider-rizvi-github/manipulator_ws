from flask import Flask, request, jsonify
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type
from ask_sdk_model.ui import SimpleCard
from ask_sdk_model import RequestEnvelope
from ask_sdk_core.serialize import DefaultSerializer

app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, how can I help!"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)
        ).set_should_end_session(False)
        return handler_input.response_builder.response


class SessionEndedRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        return handler_input.response_builder.response


sb = SkillBuilder()
sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(SessionEndedRequestHandler())
skill = sb.create()


@app.route("/alexa", methods=["POST"])
def alexa_webhook():
    envelope_dict = request.get_json(force=True)
    import json

    serializer = DefaultSerializer()

    envelope = serializer.deserialize(json.dumps(envelope_dict), RequestEnvelope)
    resp = skill.invoke(envelope, {})

    payload = serializer.serialize(resp)

    # IMPORTANT: ensure it's JSON string
    if not isinstance(payload, (str, bytes, bytearray)):
        payload = json.dumps(payload)

    return app.response_class(
        response=payload,
        status=200,
        mimetype="application/json"
    )


@app.route("/", methods=["GET"])
def health():
    return "OK", 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
