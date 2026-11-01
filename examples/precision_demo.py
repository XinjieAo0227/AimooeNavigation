"""Synthetic arithmetic example; no patient data, SDK or physical precision claim."""
import sys
import json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'AimooeNavigation'))
from AimooeLib.AimooeNavigationCore import repeatability_statistics, absolute_accuracy_statistics
points=[[0,0,0],[2,0,0]]
print(json.dumps({'units':'mm','synthetic':True,
    'repeatability':repeatability_statistics(points),
    'absolute':absolute_accuracy_statistics(points,[[0,0,0],[1,0,0]])},indent=2))
