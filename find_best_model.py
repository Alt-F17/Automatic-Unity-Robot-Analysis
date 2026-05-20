import os
import glob
import pandas as pd

def evaluate_models():
    data_dir = "TrainingData"
    # Grab all basic csv logs
    files = glob.glob(os.path.join(data_dir, '*_basic_*.csv'))
    
    if not files:
        print("No CSV files found in TrainingData.")
        return

    results = []
    
    for f in files:
        try:
            df = pd.read_csv(f)
            if df.empty:
                continue
            
            # Get the Agent ID from the filename
            filename = os.path.basename(f)
            # Example: aura_robot_basic_20260315_003218_RobotArm_-594368.csv
            parts = filename.replace('.csv', '').split('_')
            agent_id = parts[-1] 
            
            # Metrics
            final_success_rate = df['Success_Rate_Percent'].iloc[-1]
            avg_success_rate = df['Success_Rate_Percent'].mean()
            
            # Look at the last 20 episodes to measure stabilization performance (fluidity & energy)
            df_recent = df.tail(20)
            avg_energy = df_recent['Energy_Consumed'].mean()
            avg_time = df_recent['Time_Taken'].mean()
            
            results.append({
                'Agent_ID': agent_id,
                'File': filename,
                'Final_Success_%': final_success_rate,
                'Avg_Success_%': avg_success_rate,
                'Recent_Avg_Energy': avg_energy,
                'Recent_Avg_Time': avg_time
            })
        except Exception as e:
            pass

    if not results:
        print("Could not evaluate any files.")
        return
        
    df_results = pd.DataFrame(results)
    
    # Sort: Primary by Final Success %, Secondary by Lowest Energy, Tertiary by Lowest Time
    df_sorted = df_results.sort_values(by=['Final_Success_%', 'Recent_Avg_Energy', 'Recent_Avg_Time'], 
                                       ascending=[False, True, True])
    
    print("\n=== TOP 5 BEST PERFORMING AGENTS ===")
    print(df_sorted.head(5).to_string(index=False))
    
    best_agent = df_sorted.iloc[0]
    print(f"\n[WINNER] The most optimal agent is RobotArm_{best_agent['Agent_ID']}!")
    print(f"File: {best_agent['File']}")
    print(f"- Final Success Rate: {best_agent['Final_Success_%']:.2f}%")
    print(f"- Recent Avg Energy Consumed: {best_agent['Recent_Avg_Energy']:.4f}")
    print(f"- Recent Avg Time Taken: {best_agent['Recent_Avg_Time']:.4f}")

if __name__ == "__main__":
    evaluate_models()