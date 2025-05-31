#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import pickle
import numpy as np
import pandas as pd
from collections import defaultdict
import matplotlib.pyplot as plt
import seaborn as sns
import glob
from datetime import datetime
import sys

def load_cluster_analysis_json(file_path):
    """Load cluster analysis JSON file"""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    except Exception as e:
        print(f"Unable to load JSON file {file_path}: {e}")
        return None

def search_cluster_results_json(directory="."):
    """Search for cluster analysis JSON files in directory"""
    # Search pattern 1: files in top-level directory
    json_files = glob.glob(os.path.join(directory, "cluster_analysis_*.json"))
    
    # Search pattern 2: files in subdirectories
    cluster_dirs = glob.glob(os.path.join(directory, "cluster_analysis_*"))
    for d in cluster_dirs:
        if os.path.isdir(d):
            json_files.extend(glob.glob(os.path.join(d, "cluster_analysis.json")))
    
    # Sort by modification time, newest first
    json_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
    return json_files

def extract_feature_importance(cluster_data):
    """Extract feature importance from cluster data"""
    feature_importance = defaultdict(list)
    
    # Handle two-level clustering structure
    for cluster_level in ['min_clusters', 'max_clusters']:
        if cluster_level in cluster_data and cluster_data[cluster_level]:
            clusters = cluster_data[cluster_level]
            
            # Iterate through each cluster
            for i, cluster in enumerate(clusters):
                if 'key_features' in cluster:
                    # Extract key features and their importance
                    for feature_name, importance in cluster['key_features']:
                        feature_importance[feature_name].append({
                            'importance': importance,
                            'cluster_id': i+1,
                            'cluster_level': cluster_level,
                            'cluster_size': cluster.get('size', 0),
                            'main_error_type': cluster.get('main_error_type', 'unknown')
                        })
    
    return feature_importance

def analyze_global_feature_importance(feature_importance, top_k=10):
    """Analyze global feature importance"""
    # Calculate average importance and occurrence count for each feature
    feature_stats = {}
    for feature, occurrences in feature_importance.items():
        importances = [o['importance'] for o in occurrences]
        feature_stats[feature] = {
            'avg_importance': np.mean(importances),
            'max_importance': np.max(importances),
            'occurrences': len(importances),
            'details': occurrences
        }
    
    # Sort by average importance in descending order
    sorted_features = sorted(
        feature_stats.items(), 
        key=lambda x: x[1]['avg_importance'], 
        reverse=True
    )
    
    return sorted_features[:top_k]

def generate_feature_importance_plot(top_features, output_file=None):
    """Generate feature importance bar chart"""
    # Extract data
    features = [f[0] for f in top_features]
    avg_importances = [f[1]['avg_importance'] for f in top_features]
    max_importances = [f[1]['max_importance'] for f in top_features]
    
    # Create chart
    fig, ax = plt.subplots(figsize=(10, 6))
    x = range(len(features))
    width = 0.35
    
    ax.bar([i - width/2 for i in x], avg_importances, width, label='Average Importance')
    ax.bar([i + width/2 for i in x], max_importances, width, label='Maximum Importance')
    
    # Set chart style
    ax.set_xlabel('Feature Name')
    ax.set_ylabel('Importance')
    ax.set_title('Top K Feature Importance Analysis')
    ax.set_xticks(x)
    ax.set_xticklabels(features, rotation=45, ha='right')
    ax.legend()
    plt.tight_layout()
    
    # Save or display chart
    if output_file:
        plt.savefig(output_file, dpi=300)
        plt.close()
    else:
        plt.show()

def generate_feature_latex_table(top_features, output_file=None):
    """Generate feature importance LaTeX table"""
    latex_table = """\\begin{table}[h]
\\centering
\\caption{Top Feature Importance Analysis in Test Scenarios}
\\label{tab:top_features}
\\begin{tabular}{lccl}
\\toprule
Feature Name & Average Importance & Maximum Importance & Occurrence Frequency \\\\
\\midrule
"""
    
    # Fill table content
    for feature, stats in top_features:
        latex_table += f"{feature} & {stats['avg_importance']:.4f} & {stats['max_importance']:.4f} & {stats['occurrences']} \\\\\n"
    
    latex_table += """\\bottomrule
\\end{tabular}
\\end{table}"""
    
    # Write to file if specified
    if output_file:
        with open(output_file, 'w') as f:
            f.write(latex_table)
    
    return latex_table

def analyze_feature_distribution(top_features, feature_importance):
    """Analyze feature distribution across different error types"""
    error_type_distribution = {}
    
    # Only analyze top features
    top_feature_names = [f[0] for f in top_features]
    
    for feature_name in top_feature_names:
        if feature_name in feature_importance:
            # Group by error type
            error_types = {}
            for occurrence in feature_importance[feature_name]:
                error_type = occurrence['main_error_type']
                if error_type not in error_types:
                    error_types[error_type] = []
                error_types[error_type].append(occurrence['importance'])
            
            # Calculate average importance for each error type
            error_type_avg = {}
            for error_type, importances in error_types.items():
                error_type_avg[error_type] = np.mean(importances)
            
            error_type_distribution[feature_name] = error_type_avg
    
    return error_type_distribution

def generate_markdown_analysis(top_features, error_distributions, output_file=None):
    """Generate Markdown format feature analysis report"""
    markdown = f"""# Cluster Analysis Key Features Report

## Global Top {len(top_features)} Features

The table below shows the most important parameter features across all clusters:

| Rank | Feature Name | Average Importance | Maximum Importance | Occurrence Count |
|------|--------------|--------------------|--------------------|------------------|
"""
    
    for i, (feature, stats) in enumerate(top_features):
        markdown += f"| {i+1} | {feature} | {stats['avg_importance']:.4f} | {stats['max_importance']:.4f} | {stats['occurrences']} |\n"
    
    markdown += """
## Feature Distribution Across Different Error Types

The following analysis shows the importance distribution of top features across different error types:

"""
    
    for feature_name, error_dist in error_distributions.items():
        markdown += f"### {feature_name}\n\n"
        markdown += "| Error Type | Importance |\n|------------|------------|\n"
        
        sorted_errors = sorted(error_dist.items(), key=lambda x: x[1], reverse=True)
        for error_type, importance in sorted_errors:
            markdown += f"| {error_type} | {importance:.4f} |\n"
        
        markdown += "\n"
    
    markdown += """
## Conclusions and Recommendations

The above key features show significant importance in clustering across different error types, indicating that these parameters play a crucial role in determining test scenario failure modes.
Test engineers and developers should pay special attention to these important parameters and may need to design more robust control strategies or stricter input validation for them.
"""
    
    if output_file:
        with open(output_file, 'w') as f:
            f.write(markdown)
    
    return markdown

def main():
    # Check command line arguments
    if len(sys.argv) > 1:
        json_file = sys.argv[1]
    else:
        # Automatically search for the latest file
        json_files = search_cluster_results_json()
        if not json_files:
            print("No cluster analysis JSON files found")
            return
        json_file = json_files[0]
        print(f"Using latest cluster analysis file: {json_file}")
    
    # Load data
    cluster_data = load_cluster_analysis_json(json_file)
    if not cluster_data:
        return
    
    # Extract feature importance
    feature_importance = extract_feature_importance(cluster_data)
    
    # Analyze global feature importance
    top_k = 10
    top_features = analyze_global_feature_importance(feature_importance, top_k)
    
    # Save to output directory named with current timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = f"feature_analysis_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)
    
    # Print brief results
    print("\n========== Top {} Features ==========".format(top_k))
    for i, (feature, stats) in enumerate(top_features):
        print(f"{i+1}. {feature}: Average importance {stats['avg_importance']:.4f}, appears {stats['occurrences']} times")
    
    # Save results to CSV
    results_df = pd.DataFrame([
        {
            'Feature': feature,
            'AvgImportance': stats['avg_importance'],
            'MaxImportance': stats['max_importance'],
            'Occurrences': stats['occurrences']
        }
        for feature, stats in top_features
    ])
    results_df.to_csv(os.path.join(output_dir, "top_features.csv"), index=False)
    print(f"Results saved to: {os.path.join(output_dir, 'top_features.csv')}")
    
    # Generate feature importance chart
    generate_feature_importance_plot(
        top_features, 
        os.path.join(output_dir, "feature_importance_plot.png")
    )
    print(f"Feature importance chart saved to: {os.path.join(output_dir, 'feature_importance_plot.png')}")
    
    # Generate LaTeX table
    generate_feature_latex_table(
        top_features,
        os.path.join(output_dir, "feature_importance_table.tex")
    )
    print(f"LaTeX table saved to: {os.path.join(output_dir, 'feature_importance_table.tex')}")
    
    # Analyze feature distribution across different error types
    error_distributions = analyze_feature_distribution(top_features, feature_importance)
    
    # Generate Markdown analysis report
    generate_markdown_analysis(
        top_features,
        error_distributions,
        os.path.join(output_dir, "feature_analysis_report.md")
    )
    print(f"Analysis report saved to: {os.path.join(output_dir, 'feature_analysis_report.md')}")
    
    # Save complete data
    with open(os.path.join(output_dir, "feature_importance_data.json"), 'w') as f:
        json.dump({
            'top_features': [(f, {
                'avg_importance': s['avg_importance'],
                'max_importance': s['max_importance'],
                'occurrences': s['occurrences']
            }) for f, s in top_features],
            'source_file': json_file
        }, f, indent=2)

if __name__ == "__main__":
    main() 