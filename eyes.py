import pygame
from itertools import cycle

TILE_SIZE = 32
SCREEN_SIZE = pygame.Rect((0, 0, 21*TILE_SIZE, 8*TILE_SIZE))


class Expression(pygame.sprite.Sprite):
    def __init__(self, data):
        super().__init__()
        self.image = pygame.Surface((len(data[0]), len(data)))
        x = y = 0
        for row in data:
            for col in row:
                if col == "O":
                    self.image.set_at((x, y), pygame.Color('dodgerblue'))
                x += 1
            y += 1
            x = 0
        self.image = pygame.transform.scale(self.image, (TILE_SIZE*len(data[0]), TILE_SIZE*len(data)))
        self.rect = self.image.get_rect()

REGULAR = Expression([
"                     ",
"                     ",
"    OOOO     OOOO    ",
"   OOOOOO   OOOOOO   ",
"   OOOOOO   OOOOOO   ",
"    OOOO     OOOO    ",
"                     ",
"                     ",
])

QUESTION = Expression([
"                     ",
"                     ",
"    OOOO             ",
"   OOOOOO    OOOO    ",
"   OOOOOO   OOOOOO   ",
"    OOOO     OOOO    ",
"                     ",
"                     ",
])

SAD = Expression([
"                     ",
"                     ",
"                     ",
"                     ",
"                     ",
"   OOOOOO   OOOOOO   ",
"                     ",
"                     ",
])

WINK0 = Expression([
"                     ",
"                     ",
"    OOOO     OOOO    ",
"   OOOOOO   OOOOOO   ",
"   OOOOOO   OOOOOO   ",
"    OOOO     OOOO    ",
"                     ",
"                     ",
])

WINK1 = Expression([
"                     ",
"                     ",
"    OOOO             ",
"   OOOOOO    OOOO    ",
"   OOOOOO   OOOOOO   ",
"    OOOO     OOOO    ",
"                     ",
"                     ",
])

WINK2 = Expression([
"                     ",
"                     ",
"    OOOO             ",
"   OOOOOO            ",
"   OOOOOO    OOOO    ",
"    OOOO    OOOOOO   ",
"                     ",
"                     ",
])

WINK3 = Expression([
"                     ",
"                     ",
"    OOOO             ",
"   OOOOOO            ",
"   OOOOOO     OO     ",
"    OOOO    OOOOOO   ",
"                     ",
"                     ",
])

WINK4 = Expression([
"                     ",
"                     ",
"    OOOO             ",
"   OOOOOO            ",
"   OOOOOO            ",
"    OOOO    OOOOOO   ",
"                     ",
"                     ",
])


def main():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE.size)
    timer = pygame.time.Clock()
    #expressions = cycle([REGULAR, QUESTION, REGULAR, QUESTION, SAD])
    expressions = cycle([WINK0, WINK1, WINK2, WINK3, WINK4, WINK4, WINK3, WINK2, WINK1, WINK0, WINK0])
    current = next(expressions)
    pygame.time.set_timer(pygame.USEREVENT, 100)

    while True:

        for e in pygame.event.get():
            if e.type == pygame.QUIT: 
                return
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                return
            if e.type == pygame.USEREVENT:
                current = next(expressions)

        screen.fill((30, 30, 30))
        screen.blit(current.image, current.rect)
        timer.tick(60)
        pygame.display.update()

if __name__ == "__main__":
    main()