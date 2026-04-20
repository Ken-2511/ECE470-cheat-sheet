from google import genai
from google.genai import types

__all__ = ['request_llm']

# class Content(_common.BaseModel):
#   """Contains the multi-part content of a message."""

#   parts: Optional[list[Part]] = Field(
#       default=None,
#       description="""List of parts that constitute a single message. Each part may have
#       a different IANA MIME type.""",
#   )
#   role: Optional[str] = Field(
#       default=None,
#       description="""Optional. The producer of the content. Must be either 'user' or 'model'. Useful to set for multi-turn conversations, otherwise can be left blank or unset.""",
#   )


# class ContentDict(TypedDict, total=False):
#   """Contains the multi-part content of a message."""

#   parts: Optional[list[PartDict]]
#   """List of parts that constitute a single message. Each part may have
#       a different IANA MIME type."""

#   role: Optional[str]
#   """Optional. The producer of the content. Must be either 'user' or 'model'. Useful to set for multi-turn conversations, otherwise can be left blank or unset."""


# ContentOrDict = Union[Content, ContentDict]

def request_llm(model: str) -> tuple[str, dict]:
	# Extract system instructions
	system_instruction = "xxx"
	history = []
	history.append(types.Content(role=role, parts=parts))
	history.append(types.Content(role=role, parts=[types.Part.from_text(text=str(content_val))]))

	client = genai.Client()
	
	# Prepare chat configuration
	config = types.GenerateContentConfig(
		system_instruction=system_instruction.strip()
	)

	# Pop the last user message to send it
	last_user_message = xxx
	
	# Create the chat session
	chat = client.chats.create(model=model, config=config, history=history)
	
	response = chat.send_message(last_user_message.parts)

	content = response.text
	
	return content
