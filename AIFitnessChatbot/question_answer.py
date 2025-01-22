import csv 
import numpy as np 
from sklearn.feature_extraction.text import TfidfVectorizer 
from sklearn.metrics.pairwise import cosine_similarity 

#Global variables to store vectoriser and vectors 
global_vectorizer = None 
global_qa_vectors = None 

#Load questions and answers from dataset 
qa_dataset = "COMP3074-CW1-Dataset.csv" 
qa_questions = [] 
qa_answers = [] 

#Load questions and answers from dataset into their respective lists. 
def load_dataset(dataset_path): 

    """
    Load questions and answers from the dataset. 
    :param dataset_path: Path to the CSV file 
    """ 

    global global_vectorizer, global_qa_vectors 

    try:
        with open(dataset_path, 'r', encoding='utf-8') as csvfile: 
            reader = csv.reader(csvfile) 
            next(reader)  # Skip header 
            for row in reader:
                #Assuming question is in column 1, answer in column 2 
                question = row[1] 
                answer = row[2] 
                qa_questions.append(question) 
                qa_answers.append(answer) 
    except FileNotFoundError: 
        print(f"Error: Dataset file {dataset_path} not found.") 
    except IndexError: 
        print(f"Error: Invalid CSV format in {dataset_path}") 


def prepare_vectors(): 
    """
    Prepare TF-IDF vectors for all questions. 
    Sets global vectoriser and vectors. 
    """ 
    global global_vectorizer, global_qa_vectors 

    if not qa_questions:
        print("Warning: No questions loaded. Vector preparation skipped.") 
        return 
    
    global_vectorizer = TfidfVectorizer() 
    global_qa_vectors = global_vectorizer.fit_transform(qa_questions) 


def handle_question_answering(user_input): 
    """
    Find the most similar question and return its corresponding answer. 
    :param user_input: User's input question 
    :param similarity_threshold: Minimum similarity score to accept a match 
    :return: Best matching answer or a default response 
    """ 

    global global_vectorizer, global_qa_vectors 

    # Check if vectors are prepared
    if global_vectorizer is None or global_qa_vectors is None: 
        return "Sorry, I couldn't process your question at the moment." 
    
    # Vectorise user input 
    user_vector = global_vectorizer.transform([user_input]) 
    
    # Calculate similarities
    similarities = cosine_similarity(user_vector, global_qa_vectors)[0] 
    best_match_index = np.argmax(similarities) 
    best_match_similarity = similarities[best_match_index] 
    
    # Check similarity threshold
    if best_match_similarity < 0.2: 
        return "I'm sorry, I couldn't find a good answer to your question." 
    
    return qa_answers[best_match_index] 

# Load the dataset 
load_dataset(qa_dataset) 
 
# Prepare vectors 
prepare_vectors() 