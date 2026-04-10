import os
import re
import glob
import pandas as pd
import matplotlib.pyplot as plt
import questionary

def main():
    data_dir = "TrainingData"
    
    # Grab all the 'basic' csv logs
    files = glob.glob(os.path.join(data_dir, '*_basic_*.csv'))
    
    runs = {}
    for f in files:
        # Regex to grab the session ID (e.g., 20260313_121959)
        # Naming format: aura_robot_basic_20260313_121959_RobotArm_-314500.csv
        match = re.search(r'_basic_(\d{8}_\d{6})_', f)
        if match:
            session = match.group(1)
            runs.setdefault(session, []).append(f)

    if not runs:
        print(f"No valid run data found in '{data_dir}'. Make sure your unity environment has exported some CSVs!")
        return

    # 1. Ask the User which Run Date/Time they want to analyze using arrow keys
    session_choice = questionary.select(
        "Which training run would you like to analyze?",
        choices=sorted(list(runs.keys()), reverse=True),
        style=questionary.Style([('selected', 'fg:green bold')])
    ).ask()

    if not session_choice:
        print("Canceled.")
        return

    # 2. Ask the user if they want the Best Performer or the Average Fleet Behavior
    mode_choice = questionary.select(
        "Which metric would you like to graph?",
        choices=["Best Performer (Highest Final Success Rate)", "Average for all robots"],
        style=questionary.Style([('selected', 'fg:cyan bold')])
    ).ask()

    if not mode_choice:
        print("Canceled.")
        return

    session_files = runs[session_choice]
    print(f"\n[INFO] Loading {len(session_files)} robot logs for run '{session_choice}'...")

    plt.figure(figsize=(12, 7))

    # --- MODE: BEST PERFORMER ---
    if "Best Performer" in mode_choice:
        best_file = None
        best_rate = -1.0
        best_df = None
        
        for f in session_files:
            try:
                df = pd.read_csv(f)
                if 'Success_Rate_Percent' in df.columns and not df.empty:
                    # Look at the final achieved success rate of this particular robot
                    final_rate = df['Success_Rate_Percent'].iloc[-1]
                    if final_rate > best_rate:
                        best_rate = final_rate
                        best_file = f
                        best_df = df
            except Exception as e:
                print(f"Warning: Could not read {f} ({e})")
        
        if best_df is not None:
            # Extract the actual Agent ID from the CSV filename for context
            agent_match = re.search(r'RobotArm_(.*?)\.csv', best_file)
            agent_id = agent_match.group(1) if agent_match else "Unknown"
            
            plt.plot(best_df['Episode'], best_df['Success_Rate_Percent'], 
                     label=f'Top Agent: {agent_id} (Final: {best_rate:.2f}%)', 
                     color='#2ca02c', linewidth=2.5)
            plt.title(f"Success Rate Over Time - Best Performer\nRun: {session_choice}", fontsize=14, fontweight='bold')
        else:
            print("No valid data found in those records.")
            return

    # --- MODE: AVERAGE FOR ALL ROBOTS ---
    elif "Average for all robots" in mode_choice:
        all_dfs = []
        for f in session_files:
            try:
                df = pd.read_csv(f)
                if 'Success_Rate_Percent' in df.columns and not df.empty:
                    # We only care about Episode and Success Rate for the average
                    all_dfs.append(df[['Episode', 'Success_Rate_Percent']])
            except Exception as e:
                pass
                
        if all_dfs:
            # Combine all robot histories
            merged_df = pd.concat(all_dfs)
            # Group by identical episodes and average the success rate across the entire fleet
            avg_df = merged_df.groupby('Episode').mean().reset_index()
            
            plt.plot(avg_df['Episode'], avg_df['Success_Rate_Percent'], 
                     label=f'Fleet Average ({len(session_files)} robots)', 
                     color='#1f77b4', linewidth=2.5)
            
            # Optionally add a rolling smoothed line to make it look elegant!
            smoothed = avg_df['Success_Rate_Percent'].rolling(window=10, min_periods=1).mean()
            plt.plot(avg_df['Episode'], smoothed, color='orange', alpha=0.8, linestyle='--', label="Smoothed Trend")
            
            plt.title(f"Success Rate Over Time - Average Fleet Performance\nRun: {session_choice}", fontsize=14, fontweight='bold')
        else:
            print("No valid data found in those records.")
            return

    # Make the graph look beautiful
    plt.xlabel('Episode', fontsize=12)
    plt.ylabel('Success Rate (%)', fontsize=12)
    plt.ylim(-5, 105)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc="lower right", fontsize=11)
    
    # Fill under the curve
    ax = plt.gca()
    lines = ax.get_lines()
    if len(lines) > 0:
        x_data = lines[0].get_xdata()
        y_data = lines[0].get_ydata()
        ax.fill_between(x_data, y_data, alpha=0.1, color=lines[0].get_color())

    plt.tight_layout()
    
    # Save the graph to a 'graphs' folder
    os.makedirs("graphs", exist_ok=True)
    clean_mode = "best_performer" if "Best" in mode_choice else "fleet_average"
    save_path = os.path.join("graphs", f"{session_choice}_{clean_mode}.png")
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n[SUCCESS] Graph successfully saved to: {save_path}")
    
    print("[INFO] Displaying graph... (Close the window to return to console)")
    plt.show()

if __name__ == "__main__":
    main()
