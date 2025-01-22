from sklearn.feature_extraction.text import TfidfVectorizer 
from sklearn.metrics.pairwise import cosine_similarity 
from user_data import get_user_name, set_user_name  
import numpy as np 
import csv 

#Dictionary of labelled intents 
intents = { 
    'small_talk': [ 
        "Hello", "how are you", "whats up", "how's it going", "whats good", 
        "Hi there!", "greetings", "hows your day", "what's good", "hi", "whats new", 
        "Hows the weather?", "Is the weather good today?", "Hello there", "How are you?" 
    ], 
    'discoverability': [ 
        "Tell me about yourself", "What can you do?", "Explain your features", "Help", 
        "How do you work?", "What are your capabilities?", "Describe yourself", "help" 
    ], 
    'identity_management': [ 
        "What's my name?", "Who am I?", "Can you remind me of my name?", 
        "Do you know who I am?", "Tell me about myself", "My identity", 
        "What is my name?", "Whats my name", "Call me", "call me", "Change my name", "Change my name to" 
    ], 
    'class_information': [ 
        "class details", "view class information", "workouts", "view class details"  
    ], 
    'view_classes': [ 
        "View class details", "View classes", "Workout classes", "View available classes" 
    ], 
    'view_bookings': [
        "view current booking", "view booking", "view previous bookings", "previous booking", 
        "show bookings" 
    ], 
    'booking_creation': [
        "I'd like to make a booking", "Can I make a booking?", "I'd like to make a class booking", 
        "I would like to book a workout class", "Create workout class booking", "Book workout class", 
        "Create booking", "Create my booking", "Create a booking", "Make a booking", "Book a class" 
    ], 
    'booking_edit': [ 
        "Edit booking", "Can I edit my booking?", "I'd like to edit my booking", 
        "I'd like to edit my workout class booking", "Can I change my booking?", 
        "I'd like to change my booking", "Change my workout class", 
        "Change booking", "Change my booking", "Edit my booking" 
    ], 
    'booking_cancellation': [ 
        "Cancel my booking", "Can I cancel my booking?", "Delete booking" 
    ], 
    'question_answering': [] 
} 

qa_dataset = "COMP3074-CW1-Dataset.csv" 

#Add all questions into question_answering intents 
def load_qa_intents(intents, qa_dataset): 

    """ 
    Load intents from a CSV file into the question_answering intent. 
    :param intents: Dictionary of intents 
    :param qa_dataset: Path to the CSV file 
    :return: Updated intents dictionary 
    """ 

    try:
        with open(qa_dataset, 'r', encoding='utf-8') as csvfile: 
            reader = csv.reader(csvfile) 
            next(reader)  # Skip header 
            for row in reader: 
                # Assuming the question is in the second column 
                question = row[1] 
                intents['question_answering'].append(question) 
    except FileNotFoundError: 
        print(f"Warning: Dataset file {qa_dataset} not found.") 
    except IndexError: 
        print(f"Warning: Invalid CSV format in {qa_dataset}") 
    
    return intents 

#Vectorises intents and user inputs 
def prepare_intent_vectors(intents): 

    """
    Prepare TF-IDF vectorizer and transform intent texts. 
    :param intents: Dictionary of intents 
    :return: Tuple of (vectorizer, intent_vectors, intent_keys) 
    """ 

    #Combine all intent phrases 
    intent_texts = [' '.join(intent_list) for intent_list in intents.values()]
    
    #Create vectorizer 
    vectorizer = TfidfVectorizer() 
    intent_vectors = vectorizer.fit_transform(intent_texts) 
    
    #Store intent keys for easy mapping 
    intent_keys = list(intents.keys()) 
    
    return vectorizer, intent_vectors, intent_keys 

def match_intent(user_input): 

    """
    Match user input to the most similar intent. 
    :param user_input: Text input from the user 
    :param similarity_threshold: Minimum similarity score to accept an intent 
    :return: Matched intent or 'misunderstood' 
    """ 

    # Transform user input
    user_vector = vectorizer.transform([user_input]) 
    
    # Calculate cosine similarities 
    similarities = cosine_similarity(user_vector, intent_vectors)[0] 
    max_similarity = np.max(similarities) 
    
    #If low similarity, intent likely inaccurate so return misunderstood 
    if max_similarity < 0.2: #Check if similarity is above threshold 
        return "misunderstood" 
    
    # Find the index of the most similar intent 
    matched_intent_index = np.argmax(similarities) 
    return intent_keys[matched_intent_index] 

# Load QA intents from CSV
intents = load_qa_intents(intents, qa_dataset) 

# Prepare intent vectors
vectorizer, intent_vectors, intent_keys = prepare_intent_vectors(intents) 

def handle_small_talk(user_input): 
    user_input = user_input.lower()  

    if get_user_name() is None: 
        set_user_name(input("Bot: Hello! What's your name? ")) 
        return f"Nice to meet you, {get_user_name()}! How can I help you?" 
    elif user_input in ["hello", "hi", "hey", "hello there"]: 
        return f"Hi {get_user_name()}, how can I help you today?" 
    elif user_input in ["how are you", "how's it going", "How are you?", "How's it going?"]: 
        return "I'm doing well, thank you! How can I help you today?" 
    elif user_input in ["whats up", "what's up", "what's new", "what's good"]: 
        return "Not much, just helping people like you! How can I assist you?" 
    elif user_input in ["hows your day", "how's you day"]: 
        return "My day is going great, hope yours is too! How can I assist you?" 
    else:
        return f"Hi, {get_user_name()}! How can I help?"  