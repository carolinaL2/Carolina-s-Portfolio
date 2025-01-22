import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt 

# Define the test scenarios and response data
data = {
    "Tasks": [
        "Book a fitness class",
        "Change booking date", 
        "Explore functionality",
        "View booking history",
        "View available classes",
        "View single class information"
    ],
    "Completed easily": [5, 3, 2, 4, 5, 4],
    "Completed with help": [1, 2, 3, 1, 1, 2],
    "Completed with difficulty": [0, 1, 1, 1, 0, 1],
    "Could not complete": [0, 0, 0, 0, 0, 0]
} 

# Create DataFrame
df = pd.DataFrame(data).set_index('Tasks')  

# Customize the visualization style
plt.style.use('seaborn')
plt.figure(figsize=(12, 8))

# Create enhanced heatmap
sns.heatmap(df, 
            annot=True,
            cmap='RdYlGn_r',
            fmt='d',
            cbar_kws={'label': 'Number of Participants'},
            square=True)

# Customize the appearance
plt.title('Usability Testing Results: Task Completion Levels', pad=20, fontsize=14)
plt.ylabel('Test Scenarios', fontsize=12)
plt.xlabel('Completion Categories', fontsize=12)

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha='right')
plt.yticks(rotation=0) 

# Adjust layout to prevent label cutoff
plt.tight_layout()

plt.show()