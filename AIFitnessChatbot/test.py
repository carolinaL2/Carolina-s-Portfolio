from sklearn.metrics import accuracy_score, precision_score, f1_score
from intent_matching import match_intent 
import numpy as np 

test_data = [
    ("Hello there", "small_talk"),
    ("Can you change my booking?", "booking_edit"),
    ("Tell me about yourself", "discoverability"),
    ("I'd like to make a class booking", "booking_creation"),
    ("Who am I registered as?", "identity_management"),
    ("What can you do?", "discoverability"),
    ("How do I make a booking?", "booking_creation"),
    ("How do I edit my booking?", "booking_edit"),
    ("Can you tell me your functions?", "discoverability"),
    ("Good morning, how are you?", "small_talk"),
    ("I need to update my booking date", "booking_edit"),
    ("Can I book a yoga class?", "booking_creation"),
    ("What's your name?", "small_talk"),
    ("Change my name to Ellen", "identity_management"),
    ("Hi, I'm just browsing", "small_talk"),
    ("What do you do", "discoverability"),
    ("Whats the weather like", "small_talk"),
    ("I want to make a booking for tomorrow", "booking_creation"),
    ("What are stocks?", "question_answering"),
    ("Edit my class or booking", "booking_edit"),
    ("What are stocks and bonds?", "question_answering"),
    ("Hello", "small_talk")
]

# Uses the match_intent() method on each test case and gets the calculated intent
predicted_intents = [match_intent(user_input) for user_input, i in test_data]
true_intents = [true_intent for j, true_intent in test_data]

# Adding performance metrics 
accuracy = accuracy_score(true_intents, predicted_intents)
precision = precision_score(true_intents, predicted_intents, average='weighted', labels=np.unique(predicted_intents))
f1 = f1_score(true_intents, predicted_intents, average='weighted', labels=np.unique(predicted_intents))

# Outputing performance values 
print(f"Accuracy: {accuracy}\nPrecision: {precision}\nF1-Score: {f1}") 