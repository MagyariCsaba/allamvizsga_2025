from sqlalchemy import create_engine, Column, Integer, Float, DateTime
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy.exc import SQLAlchemyError
from datetime import datetime

DATABASE_URI = "mysql+pymysql://root:Torres100@127.0.0.1:3306/adatok"
engine = create_engine(DATABASE_URI)
Session = sessionmaker(bind=engine)
Base = declarative_base()


class MQTTMessage(Base):
    __tablename__ = 'mqtt_messages'
    id = Column(Integer, primary_key=True)
    timestamp = Column(DateTime, default=datetime.utcnow)
    gps_x = Column(Float)
    gps_y = Column(Float)
    gps_z = Column(Float)
    accel_x = Column(Float)
    accel_y = Column(Float)
    accel_z = Column(Float)
    gyro_x = Column(Float)
    gyro_y = Column(Float)
    gyro_z = Column(Float)


Base.metadata.create_all(engine)


class DatabaseHandler:
    def __init__(self):
        self.session = Session()

    def save_message(self, data):
        try:
            mqtt_message = MQTTMessage(
                timestamp=datetime.now(),
                gps_x=data['gpsPos'][0],
                gps_y=data['gpsPos'][1],
                gps_z=data['gpsPos'][2],
                accel_x=data['accelMeas'][0],
                accel_y=data['accelMeas'][1],
                accel_z=data['accelMeas'][2],
                gyro_x=data['gyroMeas'][0],
                gyro_y=data['gyroMeas'][1],
                gyro_z=data['gyroMeas'][2]
            )
            self.session.add(mqtt_message)
            self.session.commit()
        except SQLAlchemyError as e:
            print("Database error:", e)
            self.session.rollback()