from backend.database_handler import Session, MQTTMessage
from datetime import datetime
from sqlalchemy import and_
import json


class RouteHandler:
    def __init__(self):
        pass

    def get_route_data(self, start_time, end_time):
        session = Session()
        try:
            # Lekérdezés az időintervallum alapján
            query = session.query(MQTTMessage).filter(
                and_(
                    MQTTMessage.timestamp >= start_time,
                    MQTTMessage.timestamp <= end_time,
                    MQTTMessage.gps_x.isnot(None),
                    MQTTMessage.gps_y.isnot(None)
                )
            ).order_by(MQTTMessage.timestamp)

            results = query.all()

            # Koordináták formázása
            route_data = []
            for record in results:
                route_data.append({
                    'lat': record.gps_x,  # GPS koordináták (latitude)
                    'lon': record.gps_y,  # GPS koordináták (longitude)
                    'timestamp': record.timestamp.isoformat()
                })

            return route_data

        except Exception as e:
            print(f"Hiba az útadatok lekérdezésénél: {e}")
            return []
        finally:
            session.close()

    def parse_datetime_string(self, date_str, time_str):
        try:
            datetime_str = f"{date_str} {time_str}"
            return datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S")
        except ValueError as e:
            print(f"Hibás dátum/idő formátum: {e}")
            return None

    def get_route_for_period(self, start_date, start_time, end_date, end_time):
        start_datetime = self.parse_datetime_string(start_date, start_time)
        end_datetime = self.parse_datetime_string(end_date, end_time)

        if not start_datetime or not end_datetime:
            return []

        if start_datetime >= end_datetime:
            print("Hiba: A kezdő időpont nem lehet későbbi vagy egyenlő a befejező időpontnál")
            return []

        return self.get_route_data(start_datetime, end_datetime)