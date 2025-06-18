import os
from dotenv import load_dotenv
from sqlalchemy import create_engine, Column, Integer, Float, DateTime
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy.exc import SQLAlchemyError
from datetime import datetime

load_dotenv()

DATABASE_URI = os.getenv("DATABASE_URI", "mysql+pymysql://root:default@127.0.0.1:3306/adatok")
engine = create_engine(DATABASE_URI)
Session = sessionmaker(bind=engine)
Base = declarative_base()


class MQTTMessage(Base):
    __tablename__ = 'mqtt_messages2'
    id = Column(Integer, primary_key=True)
    timestamp = Column(DateTime, default=datetime.utcnow)
    gps_x = Column(Float)
    gps_y = Column(Float)
    gps_z = Column(Float)
    yaw = Column(Float)
    roll = Column(Float)
    pitch = Column(Float)
    yaw2 = Column(Float)
    roll2 = Column(Float)
    pitch2 = Column(Float)


Base.metadata.create_all(engine)


class DatabaseHandler:
    def __init__(self):
        self.session = Session()

    def save_message(self, data):
        try:
            mqtt_message = MQTTMessage(
                timestamp=datetime.now(),

                gps_x = data["gpsPos"][0],
                gps_y = data["gpsPos"][1],
                gps_z = data["gpsPos"][2],

                yaw = data["imuAngles"][0],
                roll = data["imuAngles"][1],
                pitch = data["imuAngles"][2],

                yaw2 = data["imu2Angles"][0],
                roll2 = data["imu2Angles"][1],
                pitch2 = data["imu2Angles"][2]
                )
            self.session.add(mqtt_message)
            self.session.commit()
        except SQLAlchemyError as e:
            print("Database error:", e)
            self.session.rollback()
        finally:
            self.session.close()


