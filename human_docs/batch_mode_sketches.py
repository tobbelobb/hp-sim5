from PIL import Image, ImageDraw, ImageFont

def create_wireframe(title, type="setup"):
    # Canvas settings (Dark Mode)
    width, height = 1200, 800
    bg_color = (20, 22, 31) # Dark blue-grey
    panel_color = (30, 34, 46)
    accent_color = (255, 100, 100) # Pinkish red from original
    text_color = (220, 220, 220)
    highlight_color = (100, 200, 100) # Green

    img = Image.new('RGB', (width, height), bg_color)
    draw = ImageDraw.Draw(img)
    
    # Draw Header
    draw.rectangle([(0,0), (width, 60)], fill=panel_color)
    draw.text((20, 20), "HANGPRINTER SIM // " + title, fill=text_color)

    if type == "setup":
        # SETUP VIEW WIREFRAME
        
        # Left Panel: Buckets
        draw.text((50, 100), "1. UPLOAD RESOURCES", fill=text_color)
        
        # Bucket 1: Machines
        draw.rectangle([(50, 140), (350, 340)], outline=text_color, width=2)
        draw.text((70, 160), "DROP MACHINES HERE\n(.usda)", fill=text_color)
        draw.rectangle([(70, 200), (330, 240)], fill=panel_color)
        draw.text((80, 215), " hexagon_v2.usda", fill=text_color)
        draw.rectangle([(70, 250), (330, 290)], fill=panel_color)
        draw.text((80, 265), " original_v1.usda", fill=text_color)
        
        # Bucket 2: Inputs
        draw.rectangle([(400, 140), (700, 340)], outline=text_color, width=2)
        draw.text((420, 160), "DROP INPUTS HERE\n(.serial, .can)", fill=text_color)
        draw.rectangle([(420, 200), (680, 240)], fill=panel_color)
        draw.text((430, 215), " calibration_print.serial", fill=text_color)
        
        # Right Panel: The Manifest
        draw.text((50, 400), "2. EXPERIMENT MANIFEST (Preview)", fill=text_color)
        draw.rectangle([(50, 440), (1100, 700)], fill=panel_color)
        
        # Table Header
        draw.text((70, 460), "ID    MACHINE          INPUT              EST. TIME", fill=text_color)
        draw.line([(50, 490), (1100, 490)], fill=text_color)
        
        # Rows
        draw.text((70, 510), "01    hexagon_v2       calibration.serial  ~2m", fill=text_color)
        draw.text((70, 550), "02    original_v1      calibration.serial  ~2m", fill=text_color)
        
        # Action Button
        draw.rectangle([(900, 720), (1100, 770)], fill=accent_color)
        draw.text((940, 740), "RUN BATCH (2)", fill=(255,255,255))

    elif type == "dashboard":
        # DASHBOARD VIEW WIREFRAME
        
        # Top Panel: Score Tuner
        draw.rectangle([(50, 80), (1150, 200)], fill=panel_color)
        draw.text((70, 100), "QC SCORE WEIGHTING (Toggle components to re-rank)", fill=text_color)
        
        # Toggles
        draw.rectangle([(70, 140), (90, 160)], fill=highlight_color)
        draw.text((100, 140), "Position Error", fill=text_color)
        
        draw.rectangle([(250, 140), (270, 160)], fill=highlight_color)
        draw.text((280, 140), "Slack Lines", fill=text_color)
        
        draw.rectangle([(430, 140), (450, 160)], outline=text_color) # Unchecked
        draw.text((460, 140), "Torque Limit (Ignored)", fill=(150,150,150))

        # Main Content: Split View
        
        # Left: Leaderboard
        draw.rectangle([(50, 230), (600, 750)], fill=panel_color)
        draw.text((70, 250), "LEADERBOARD (Sorted by QC Score)", fill=text_color)
        draw.line([(50, 280), (600, 280)], fill=text_color)
        
        # Header
        draw.text((60, 290), "Rank  Run Name         Score", fill=text_color)
        # Row 1 (Selected)
        draw.rectangle([(55, 310), (595, 340)], fill=accent_color)
        draw.text((60, 315), "#1    Hexagon_Calib    98.5/100", fill=(255,255,255))
        # Row 2
        draw.text((60, 355), "#2    Original_Calib   82.0/100", fill=text_color)
        
        # Right: Drill Down Details
        draw.text((650, 230), "SELECTED RUN DETAILS", fill=text_color)
        
        # Graph Placeholder
        draw.rectangle([(650, 260), (1150, 500)], outline=text_color)
        draw.line([(660, 480), (1140, 480)], fill=text_color) # X axis
        draw.line([(660, 270), (660, 480)], fill=text_color) # Y axis
        # Draw a fake graph line
        points = [(660, 480), (700, 470), (750, 300), (800, 450), (900, 460), (1140, 480)]
        draw.line(points, fill=highlight_color, width=2)
        draw.text((760, 280), "⚠️ Max Error at 00:45s", fill=highlight_color)
        
        # Replay Button
        draw.rectangle([(650, 550), (900, 600)], outline=accent_color)
        draw.text((680, 570), "WATCH REPLAY IN 3D", fill=accent_color)

    return img

# Generate the two images
setup_img = create_wireframe("EXPERIMENT SETUP", "setup")
dash_img = create_wireframe("RESULTS DASHBOARD", "dashboard")

# Save files
setup_path = "ux_setup_wireframe.png"
dash_path = "ux_dashboard_wireframe.png"
setup_img.save(setup_path)
dash_img.save(dash_path)

print(setup_path, dash_path)
