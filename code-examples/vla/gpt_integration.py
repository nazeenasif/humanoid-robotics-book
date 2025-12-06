import asyncio
import json
import openai
from typing import Dict, List, Optional, Any
import logging
from dataclasses import dataclass

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class RobotAction:
    """Represents an action that the robot can perform"""
    name: str
    parameters: Dict[str, Any]
    description: str

@dataclass
class VisionObservation:
    """Represents visual information from the robot's sensors"""
    objects: List[Dict[str, Any]]
    spatial_relationships: List[Dict[str, Any]]
    environment_description: str

class GPTVLAIntegrator:
    """
    Integrates OpenAI GPT with Vision-Language-Action system for humanoid robots
    """

    def __init__(self, api_key: str, model: str = "gpt-4-turbo"):
        """
        Initialize the GPT VLA integrator

        Args:
            api_key: OpenAI API key
            model: GPT model to use
        """
        openai.api_key = api_key
        self.model = model
        self.action_space = self._define_action_space()

    def _define_action_space(self) -> Dict[str, RobotAction]:
        """
        Define the available actions for the humanoid robot
        """
        return {
            "move_to": RobotAction(
                name="move_to",
                parameters={"location": "str", "relative_to": "str"},
                description="Move the robot to a specified location"
            ),
            "pick_up": RobotAction(
                name="pick_up",
                parameters={"object": "str", "location": "str"},
                description="Pick up an object from a location"
            ),
            "place": RobotAction(
                name="place",
                parameters={"object": "str", "location": "str"},
                description="Place an object at a location"
            ),
            "grasp": RobotAction(
                name="grasp",
                parameters={"object": "str", "grasp_type": "str"},
                description="Grasp an object with specified grasp type"
            ),
            "release": RobotAction(
                name="release",
                parameters={"object": "str"},
                description="Release a grasped object"
            ),
            "look_at": RobotAction(
                name="look_at",
                parameters={"target": "str"},
                description="Direct the robot's vision system to look at a target"
            ),
            "point_to": RobotAction(
                name="point_to",
                parameters={"target": "str"},
                description="Point to a specific object or location"
            ),
            "wave": RobotAction(
                name="wave",
                parameters={},
                description="Perform a waving gesture"
            ),
            "speak": RobotAction(
                name="speak",
                parameters={"text": "str"},
                description="Speak the provided text aloud"
            ),
            "wait": RobotAction(
                name="wait",
                parameters={"duration": "float"},
                description="Wait for a specified duration in seconds"
            )
        }

    async def process_language_command(self,
                                     command: str,
                                     vision_observation: VisionObservation,
                                     context: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Process a natural language command using GPT and return executable actions

        Args:
            command: Natural language command from user
            vision_observation: Current visual information from robot
            context: Additional context information

        Returns:
            List of actions to execute
        """
        try:
            # Create a detailed prompt for GPT
            prompt = self._create_vla_prompt(command, vision_observation, context)

            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": self._get_system_prompt()
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                functions=[
                    {
                        "name": "execute_action",
                        "description": "Execute a robot action",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "action": {
                                    "type": "string",
                                    "enum": list(self.action_space.keys()),
                                    "description": "The action to execute"
                                },
                                "parameters": {
                                    "type": "object",
                                    "description": "Parameters for the action"
                                }
                            },
                            "required": ["action", "parameters"]
                        }
                    }
                ],
                function_call="auto"
            )

            # Parse the response
            actions = []
            for choice in response.choices:
                message = choice.message
                if message.get("function_call"):
                    function_call = message["function_call"]
                    action_data = json.loads(function_call["arguments"])
                    actions.append(action_data)
                elif message.get("content"):
                    # If GPT responds with text instead of function call, convert to speak action
                    actions.append({
                        "action": "speak",
                        "parameters": {"text": message["content"]}
                    })

            logger.info(f"Generated actions for command '{command}': {actions}")
            return actions

        except Exception as e:
            logger.error(f"Error processing language command: {e}")
            # Return a safe fallback action
            return [{
                "action": "speak",
                "parameters": {"text": f"I'm sorry, I encountered an error processing your command: {str(e)}"}
            }]

    def _create_vla_prompt(self,
                          command: str,
                          vision_observation: VisionObservation,
                          context: Optional[Dict] = None) -> str:
        """
        Create a detailed prompt combining language command and visual information
        """
        prompt = f"Human command: {command}\n\n"

        prompt += "Current visual observation:\n"
        prompt += f"Objects detected: {', '.join([obj['name'] for obj in vision_observation.objects])}\n"
        prompt += f"Environment: {vision_observation.environment_description}\n"

        if vision_observation.spatial_relationships:
            prompt += "Spatial relationships:\n"
            for rel in vision_observation.spatial_relationships:
                prompt += f"  - {rel['subject']} is {rel['relationship']} {rel['object']}\n"

        if context:
            prompt += f"\nAdditional context: {context}\n"

        prompt += f"\nAvailable actions: {list(self.action_space.keys())}\n"
        prompt += "Please break down the human command into executable robot actions."

        return prompt

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt that guides GPT's behavior
        """
        return """
        You are an AI assistant for a humanoid robot with Vision-Language-Action capabilities.
        Your role is to interpret human commands and convert them into a sequence of actions
        that the robot can execute. Consider the visual information provided when interpreting
        the command. Be precise with object references and spatial relationships. If the
        command is ambiguous, ask for clarification by generating a 'speak' action with
        a clarifying question. Always ensure actions are safe and appropriate for the context.
        """

    def validate_action(self, action: Dict[str, Any]) -> bool:
        """
        Validate if an action is valid for the robot
        """
        if "action" not in action:
            return False

        action_name = action["action"]
        if action_name not in self.action_space:
            logger.warning(f"Unknown action: {action_name}")
            return False

        # Validate parameters
        required_params = list(self.action_space[action_name].parameters.keys())
        if "parameters" not in action:
            action["parameters"] = {}

        # Check if required parameters are present
        for param in required_params:
            if param not in action["parameters"]:
                logger.warning(f"Missing required parameter '{param}' for action '{action_name}'")
                return False

        return True

    async def execute_command_sequence(self,
                                     command: str,
                                     vision_observation: VisionObservation,
                                     context: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Execute a complete command sequence: process command, validate actions, return executable sequence
        """
        # Process the command to get action sequence
        raw_actions = await self.process_language_command(command, vision_observation, context)

        # Validate and filter actions
        validated_actions = []
        for action in raw_actions:
            if self.validate_action(action):
                validated_actions.append(action)
            else:
                logger.warning(f"Invalid action skipped: {action}")

        return validated_actions

# Example usage
async def main():
    """
    Example usage of the GPT VLA integrator
    """
    # Note: In a real implementation, you would use a real API key
    # This is just a demonstration
    integrator = GPTVLAIntegrator(api_key="YOUR_OPENAI_API_KEY_HERE")

    # Example vision observation
    vision_obs = VisionObservation(
        objects=[
            {"name": "red cup", "position": [0.5, 0.3, 0.8], "color": "red"},
            {"name": "blue mug", "position": [0.7, 0.1, 0.8], "color": "blue"},
            {"name": "wooden table", "position": [0.6, 0.2, 0.75], "type": "furniture"}
        ],
        spatial_relationships=[
            {"subject": "red cup", "relationship": "on", "object": "wooden table"},
            {"subject": "blue mug", "relationship": "next to", "object": "red cup"}
        ],
        environment_description="Kitchen environment with table and chairs"
    )

    # Example commands to test
    commands = [
        "Pick up the red cup from the table",
        "Move to the kitchen counter",
        "Wave to me",
        "Tell me what objects you see"
    ]

    for command in commands:
        print(f"\nProcessing command: '{command}'")
        actions = await integrator.execute_command_sequence(command, vision_obs)
        print(f"Generated actions: {actions}")

if __name__ == "__main__":
    # Run the example
    # Note: This requires an active OpenAI API key to work properly
    asyncio.run(main())