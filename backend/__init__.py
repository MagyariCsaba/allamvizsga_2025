import os
from dotenv import load_dotenv

# Betölti a .env fájlt
load_dotenv()

# Elérhetővé teszi a változókat
DATABASE_URI = os.getenv("DATABASE_URI")