#!/usr/bin/env python3
"""
Script to create a Jupyter notebook for radar data visualization.
Run this to generate plot_radar_data.ipynb
"""

import json

notebook = {
    "cells": [
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "# Radar Data Analysis\n",
                "## Visualization of radar parameters over time\n",
                "\n",
                "This notebook loads CSV files with radar data and plots each parameter as a function of time."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "import pandas as pd\n",
                "import matplotlib.pyplot as plt\n",
                "import numpy as np\n",
                "import glob\n",
                "import os\n",
                "\n",
                "# Set plot style\n",
                "plt.style.use('seaborn-v0_8-darkgrid')\n",
                "plt.rcParams['figure.figsize'] = (14, 8)"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## Load Data"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Get current directory\n",
                "data_dir = os.getcwd()\n",
                "print(f\"Looking for CSV files in: {data_dir}\")\n",
                "\n",
                "# Find latest CSV files for each parameter\n",
                "parameters = ['rcs', 'velocity', 'x', 'y', 'z', 'snr']\n",
                "data = {}\n",
                "\n",
                "for param in parameters:\n",
                "    files = glob.glob(os.path.join(data_dir, f'*{param}_*.csv'))\n",
                "    if files:\n",
                "        # Get most recent file\n",
                "        latest_file = max(files, key=os.path.getmtime)\n",
                "        df = pd.read_csv(latest_file)\n",
                "        # Convert timestamp to relative time\n",
                "        if len(df) > 0:\n",
                "            df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]\n",
                "            data[param] = df\n",
                "            print(f\"{param}: {len(df)} points, {os.path.basename(latest_file)}\")\n",
                "\n",
                "print(f\"\\nLoaded {len(data)} parameters\")"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## Plot All Parameters"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Parameter info for plotting\n",
                "param_info = {\n",
                "    'rcs': {'label': 'RCS (dBsm)', 'color': 'red'},\n",
                "    'velocity': {'label': 'Velocity (m/s)', 'color': 'blue'},\n",
                "    'x': {'label': 'X Position (m)', 'color': 'green'},\n",
                "    'y': {'label': 'Y Position (m)', 'color': 'orange'},\n",
                "    'z': {'label': 'Z Position (m)', 'color': 'purple'},\n",
                "    'snr': {'label': 'SNR (dB)', 'color': 'brown'}\n",
                "}\n",
                "\n",
                "# Create subplots\n",
                "if data:\n",
                "    fig, axes = plt.subplots(len(data), 1, figsize=(14, 4*len(data)))\n",
                "    if len(data) == 1:\n",
                "        axes = [axes]\n",
                "    \n",
                "    for idx, (param, df) in enumerate(data.items()):\n",
                "        ax = axes[idx]\n",
                "        info = param_info[param]\n",
                "        \n",
                "        ax.plot(df['time_relative'], df[param], \n",
                "                color=info['color'], alpha=0.6, linewidth=0.5, marker='.', markersize=2)\n",
                "        \n",
                "        ax.set_xlabel('Time (s)', fontsize=12)\n",
                "        ax.set_ylabel(info['label'], fontsize=12)\n",
                "        ax.set_title(f\"{info['label']} over Time\", fontsize=14, fontweight='bold')\n",
                "        ax.grid(True, alpha=0.3)\n",
                "        \n",
                "        # Stats\n",
                "        stats = f\"Mean: {df[param].mean():.2f}\\\\nStd: {df[param].std():.2f}\\\\nMin: {df[param].min():.2f}\\\\nMax: {df[param].max():.2f}\"\n",
                "        ax.text(0.02, 0.98, stats, transform=ax.transAxes, fontsize=10, \n",
                "                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))\n",
                "    \n",
                "    plt.tight_layout()\n",
                "    plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## RCS Analysis"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "if 'rcs' in data:\n",
                "    df = data['rcs']\n",
                "    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))\n",
                "    \n",
                "    ax1.plot(df['time_relative'], df['rcs'], color='red', alpha=0.6)\n",
                "    ax1.set_xlabel('Time (s)')\n",
                "    ax1.set_ylabel('RCS (dBsm)')\n",
                "    ax1.set_title('RCS over Time')\n",
                "    ax1.grid(True, alpha=0.3)\n",
                "    \n",
                "    ax2.hist(df['rcs'], bins=50, color='red', alpha=0.7, edgecolor='black')\n",
                "    ax2.set_xlabel('RCS (dBsm)')\n",
                "    ax2.set_ylabel('Frequency')\n",
                "    ax2.set_title('RCS Distribution')\n",
                "    ax2.grid(True, alpha=0.3)\n",
                "    \n",
                "    plt.tight_layout()\n",
                "    plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## Velocity Analysis"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "if 'velocity' in data:\n",
                "    df = data['velocity']\n",
                "    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))\n",
                "    \n",
                "    ax1.plot(df['time_relative'], df['velocity'], color='blue', alpha=0.6)\n",
                "    ax1.axhline(y=0, color='black', linestyle='--', alpha=0.5)\n",
                "    ax1.set_xlabel('Time (s)')\n",
                "    ax1.set_ylabel('Velocity (m/s)')\n",
                "    ax1.set_title('Velocity over Time')\n",
                "    ax1.grid(True, alpha=0.3)\n",
                "    \n",
                "    ax2.hist(df['velocity'], bins=50, color='blue', alpha=0.7, edgecolor='black')\n",
                "    ax2.axvline(x=0, color='black', linestyle='--', alpha=0.5)\n",
                "    ax2.set_xlabel('Velocity (m/s)')\n",
                "    ax2.set_ylabel('Frequency')\n",
                "    ax2.set_title('Velocity Distribution')\n",
                "    ax2.grid(True, alpha=0.3)\n",
                "    \n",
                "    plt.tight_layout()\n",
                "    plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## 3D Position (X, Y, Z)"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "if all(p in data for p in ['x', 'y', 'z']):\n",
                "    fig, axes = plt.subplots(2, 2, figsize=(16, 10))\n",
                "    \n",
                "    axes[0,0].plot(data['x']['time_relative'], data['x']['x'], 'g-', alpha=0.6)\n",
                "    axes[0,0].set_xlabel('Time (s)')\n",
                "    axes[0,0].set_ylabel('X (m)')\n",
                "    axes[0,0].set_title('X Position')\n",
                "    axes[0,0].grid(True, alpha=0.3)\n",
                "    \n",
                "    axes[0,1].plot(data['y']['time_relative'], data['y']['y'], color='orange', alpha=0.6)\n",
                "    axes[0,1].set_xlabel('Time (s)')\n",
                "    axes[0,1].set_ylabel('Y (m)')\n",
                "    axes[0,1].set_title('Y Position')\n",
                "    axes[0,1].grid(True, alpha=0.3)\n",
                "    \n",
                "    axes[1,0].plot(data['z']['time_relative'], data['z']['z'], color='purple', alpha=0.6)\n",
                "    axes[1,0].set_xlabel('Time (s)')\n",
                "    axes[1,0].set_ylabel('Z (m)')\n",
                "    axes[1,0].set_title('Z Position')\n",
                "    axes[1,0].grid(True, alpha=0.3)\n",
                "    \n",
                "    sc = axes[1,1].scatter(data['x']['x'], data['y']['y'], \n",
                "                           c=data['x']['time_relative'], cmap='viridis', alpha=0.6, s=1)\n",
                "    axes[1,1].set_xlabel('X (m)')\n",
                "    axes[1,1].set_ylabel('Y (m)')\n",
                "    axes[1,1].set_title('XY Position (colored by time)')\n",
                "    axes[1,1].set_aspect('equal')\n",
                "    axes[1,1].grid(True, alpha=0.3)\n",
                "    plt.colorbar(sc, ax=axes[1,1], label='Time (s)')\n",
                "    \n",
                "    plt.tight_layout()\n",
                "    plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["## Data Statistics"]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "if data:\n",
                "    summary = {}\n",
                "    for param, df in data.items():\n",
                "        summary[param] = {\n",
                "            'Count': len(df),\n",
                "            'Mean': df[param].mean(),\n",
                "            'Std': df[param].std(),\n",
                "            'Min': df[param].min(),\n",
                "            'Max': df[param].max(),\n",
                "            'Median': df[param].median()\n",
                "        }\n",
                "    \n",
                "    summary_df = pd.DataFrame(summary).T\n",
                "    print(\"RADAR DATA SUMMARY STATISTICS\")\n",
                "    print(\"=\"*80)\n",
                "    print(summary_df.to_string())\n",
                "    print(\"=\"*80)"
            ]
        }
    ],
    "metadata": {
        "kernelspec": {
            "display_name": "Python 3",
            "language": "python",
            "name": "python3"
        },
        "language_info": {
            "codemirror_mode": {"name": "ipython", "version": 3},
            "file_extension": ".py",
            "mimetype": "text/x-python",
            "name": "python",
            "nbconvert_exporter": "python",
            "pygments_lexer": "ipython3",
            "version": "3.10.0"
        }
    },
    "nbformat": 4,
    "nbformat_minor": 4
}

# Write notebook
with open('plot_radar_data.ipynb', 'w') as f:
    json.dump(notebook, f, indent=2)

print("Notebook created: plot_radar_data.ipynb")

