import os, sys, csv, sqlite3
from sql_statements import *

class HospitalityDBHandlerNode(object):
    def __init__(self, db_name):
        db_existed = os.path.exists(db_name)
        self.db = sqlite3.connect(db_name)
        self.cursor = self.db.cursor()

        if db_existed:
            if not self.verify_db():
                self.init_db()
        else:
            self.init_db()
    
    def verify_db(self):
        pass

    def init_db(self):
        self.cursor.execute(CREATE_TABLE_PLACES)
        self.cursor.execute(CREATE_TABLE_PATIENTS)
        self.cursor.execute(CREATE_TABLE_DOCTORS)
        self.cursor.execute(CREATE_TABLE_SCHEDULES)
        self.cursor.execute(CREATE_TABLE_ASSETS)
        self.cursor.execute(CREATE_TABLE_SYMPTOMS)
        self.cursor.execute(CREATE_TABLE_MEDICATIONS)
        self.cursor.execute(CREATE_TABLE_MEAL_SCHEDULES)
        self.db.commit()
        
    
    def get_iterative_task(self, start_time, end_time):
        pass