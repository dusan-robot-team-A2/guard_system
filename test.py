import asyncio

async def a():
    print("init a")
    
    asyncio.create_task(b())
    print("end a")

async def b():
    print("init b")
    await asyncio.sleep(2)
    print("end b")

asyncio.run(a())
print("end program")