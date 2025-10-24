# benchmarks_plot.py
import pandas as pd
import matplotlib.pyplot as plt

# Load benchmark data
results = pd.read_csv('benchmark_results.csv')

# Boxplot for computation time
plt.figure(figsize=(8,6))
results.boxplot(column='time', by='planner')
plt.title('Computation Time by Planner')
plt.ylabel('Time (seconds)')
plt.savefig('boxplot_time.png')
plt.show()

# Boxplot for tree size
plt.figure(figsize=(8,6))
results.boxplot(column='tree_size', by='planner')
plt.title('Tree Size by Planner')
plt.ylabel('Tree Size (total waypoints)')
plt.savefig('boxplot_tree_size.png')
plt.show()

print('Boxplots saved as boxplot_time.png and boxplot_tree_size.png')
